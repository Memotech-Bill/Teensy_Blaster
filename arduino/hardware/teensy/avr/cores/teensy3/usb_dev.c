/* Teensyduino Core Library
 * http://www.pjrc.com/teensy/
 * Copyright (c) 2017 PJRC.COM, LLC.
 *
 * Permission is hereby granted, free of charge, to any person obtaining
 * a copy of this software and associated documentation files (the
 * "Software"), to deal in the Software without restriction, including
 * without limitation the rights to use, copy, modify, merge, publish,
 * distribute, sublicense, and/or sell copies of the Software, and to
 * permit persons to whom the Software is furnished to do so, subject to
 * the following conditions:
 *
 * 1. The above copyright notice and this permission notice shall be
 * included in all copies or substantial portions of the Software.
 *
 * 2. If the Software is incorporated into a build system that allows
 * selection among a list of target devices, then similar target
 * devices manufactured by PJRC.COM must be included in the list of
 * target devices and selectable in the same manner.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS
 * BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN
 * ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 *
 * Trying to understand this rather complex code?
 *
 * Kevin Cuzner wrote a simpler version, and a great blog article:
 *   http://kevincuzner.com/2014/12/12/teensy-3-1-bare-metal-writing-a-usb-driver/
 *   https://github.com/kcuzner/teensy-oscilloscope/blob/master/scope-teensy/src/usb.c
 *
 * Andy Payne wrote another relatively simple USB example for Freescale Kinetis
 *   https://github.com/payne92/bare-metal-arm
 */

#include "usb_dev.h"
#if F_CPU >= 20000000 && defined(NUM_ENDPOINTS)

#include "kinetis.h"
//#include "HardwareSerial.h"
#include "usb_mem.h"
#include <string.h> // for memset

// This code has a known bug with compiled with -O2 optimization on gcc 5.4.1
// https://forum.pjrc.com/threads/53574-Teensyduino-1-43-Beta-2?p=186177&viewfull=1#post186177
#if defined(__MKL26Z64__)
#pragma GCC optimize ("Os")
#else
#pragma GCC optimize ("O3")
#endif

// buffer descriptor table - Required by hardware, see 46.3.4 in Teensy 3.5 hardware manual

typedef struct {
        uint32_t desc;
        void * addr;
} bdt_t;

// Four buffer descriptors per endpoint:
// 0. Even receive buffer
// 1. Odd receive buffer
// 2. Even transmit buffer
// 3. Odd transmit buffer

__attribute__ ((section(".usbdescriptortable"), used))
static bdt_t table[(NUM_ENDPOINTS+1)*4];

static usb_packet_t *rx_first[NUM_ENDPOINTS];
static usb_packet_t *rx_last[NUM_ENDPOINTS];
static usb_packet_t *tx_first[NUM_ENDPOINTS];
static usb_packet_t *tx_last[NUM_ENDPOINTS];
uint16_t usb_rx_byte_count_data[NUM_ENDPOINTS];

static uint8_t tx_state[NUM_ENDPOINTS];
#define TX_STATE_BOTH_FREE_EVEN_FIRST   0
#define TX_STATE_BOTH_FREE_ODD_FIRST    1
#define TX_STATE_EVEN_FREE              2
#define TX_STATE_ODD_FREE               3
#define TX_STATE_NONE_FREE_EVEN_FIRST   4
#define TX_STATE_NONE_FREE_ODD_FIRST    5

#define BDT_OWN         0x80    // Set to enable hardware to access buffer descriptor
#define BDT_DATA1       0x40    // DATA1 field - Set this for odd buffer descriptors
#define BDT_DATA0       0x00    // DATA0 field - Set this for even buffer descriptors
#define BDT_DTS         0x08    // Enable Data Toggle Synchronization
#define BDT_STALL       0x04    // Stall if using this buffer descriptor
#define BDT_PID(n)      (((n) >> 2) & 15)   // Token PID: 0x1 = OUT, 0x9 = IN, 0xD = Setup

// Sets the buffer descriptor:
//    Sets the data length
//    Sets the DATA0 / DATA1 bit - In practice use bit 2 of BD address to set this.
//    Enables Data Toggle Synchronization
//    Hands ownership of the buffer to the USB hardware
#define BDT_DESC(count, data)   (BDT_OWN | BDT_DTS \
                                | ((data) ? BDT_DATA1 : BDT_DATA0) \
                                | ((count) << 16))

#define TX   1
#define RX   0
#define ODD  1
#define EVEN 0
#define DATA0 0
#define DATA1 1
// Index into the BD table as a function of endpoint, tx/rx, and odd/even.
#define index(endpoint, tx, odd) (((endpoint) << 2) | ((tx) << 1) | (odd))

// Get BD address from contents of USB0_STAT register - 46.4.13 in hardware manual
#define stat2bufferdescriptor(stat) (table + ((stat) >> 2))


static union {
 struct {
  union {
   struct {
        uint8_t bmRequestType;
        uint8_t bRequest;
   };
        uint16_t wRequestAndType;
  };
        uint16_t wValue;
        uint16_t wIndex;
        uint16_t wLength;
 };
 struct {
        uint32_t word1;
        uint32_t word2;
 };
} setup;


#define GET_STATUS              0
#define CLEAR_FEATURE           1
#define SET_FEATURE             3
#define SET_ADDRESS             5
#define GET_DESCRIPTOR          6
#define SET_DESCRIPTOR          7
#define GET_CONFIGURATION       8
#define SET_CONFIGURATION       9
#define GET_INTERFACE           10
#define SET_INTERFACE           11
#define SYNCH_FRAME             12

// SETUP always uses a DATA0 PID for the data field of the SETUP transaction.
// transactions in the data phase start with DATA1 and toggle (figure 8-12, USB1.1)
// Status stage uses a DATA1 PID.

static uint8_t ep0_rx0_buf[EP0_SIZE] __attribute__ ((aligned (4)));
static uint8_t ep0_rx1_buf[EP0_SIZE] __attribute__ ((aligned (4)));
static const uint8_t *ep0_tx_ptr = NULL;
static uint16_t ep0_tx_len;
static uint8_t ep0_tx_bdt_bank = 0;
static uint8_t ep0_tx_data_toggle = 0;
#ifdef USB_POOL
uint8_t usb_rx_memory_needed[NUM_ENDPOINTS];
#else
uint8_t usb_rx_memory_needed = 0;
#endif

volatile uint8_t usb_configuration = 0;
volatile uint8_t usb_reboot_timer = 0;

#if MEM_DEBUG > 0
void usb_queues(void)
    {
    const usb_packet_t *pbase = usb_mem_base();
    for (int iEP = 0; iEP < NUM_ENDPOINTS; ++iEP)
        {
        UsbLog("EP %d TX Queue:", iEP+1);
        usb_packet_t *p = tx_first[iEP];
        while (p != NULL)
            {
            UsbLog(" %d (%d)", p - pbase, p->len);
            p = p->next;
            }
        UsbLog("\r\n");
        UsbLog("EP %d RX Queue:", iEP+1);
        p = rx_first[iEP];
        while (p != NULL)
            {
            UsbLog(" %d (%d)", p - pbase, p->len);
            p = p->next;
            }
        UsbLog("\r\n");
        }
    for (int i = 0; i <= NUM_ENDPOINTS; ++i)
        {
        for (int j = 0; j < 4; ++j)
            UsbLog (" 0x%08X:%p", table[4*i+j].desc, table[4*i+j].addr);
        UsbLog ("\r\n");
        }
    }
#endif

static void endpoint0_stall(void)
{
        USB0_ENDPT0 = USB_ENDPT_EPSTALL | USB_ENDPT_EPRXEN | USB_ENDPT_EPTXEN | USB_ENDPT_EPHSHK;
}


static void endpoint0_transmit(const void *data, uint32_t len)
{
#if 0
        serial_print("tx0:");
        serial_phex32((uint32_t)data);
        serial_print(",");
        serial_phex16(len);
        serial_print(ep0_tx_bdt_bank ? ", odd" : ", even");
        serial_print(ep0_tx_data_toggle ? ", d1\n" : ", d0\n");
#endif
        table[index(0, TX, ep0_tx_bdt_bank)].addr = (void *)data;
        table[index(0, TX, ep0_tx_bdt_bank)].desc = BDT_DESC(len, ep0_tx_data_toggle);
        ep0_tx_data_toggle ^= 1;
        ep0_tx_bdt_bank ^= 1;
}

static uint8_t reply_buffer[8];

static void usb_setup(void)
{
        const uint8_t *data = NULL;
        uint32_t datalen = 0;
        const usb_descriptor_list_t *list;
        uint32_t size;
        volatile uint8_t *reg;
        uint8_t epconf;
        const uint8_t *cfg;
        int i;
        // UsbLog ("Setup: 0x%04X\r\n", setup.wRequestAndType);
        switch (setup.wRequestAndType) {
          case 0x0500: // SET_ADDRESS
                break;
          case 0x0900: // SET_CONFIGURATION
                //serial_print("configure\n");
                usb_configuration = setup.wValue;
                reg = &USB0_ENDPT1;
                cfg = usb_endpoint_config_table;
                // clear all BDT entries, free any allocated memory...
                for (i=4; i < (NUM_ENDPOINTS+1)*4; i++) {
                        if (table[i].desc & BDT_OWN) {
                                usb_packet_t *p = (usb_packet_t *)((uint8_t *)(table[i].addr)
                                    - offsetof(usb_packet_t, buf));
#if MEM_DEBUG > 0
                                usb_free(p, i);
#else
                                usb_free(p);
#endif
                        }
                }
                // free all queued packets
                for (i=0; i < NUM_ENDPOINTS; i++) {
                        usb_packet_t *p, *n;
                        p = rx_first[i];
                        while (p) {
                                n = p->next;
#if MEM_DEBUG > 0
                                usb_free(p, __LINE__);
#else
                                usb_free(p);
#endif
                                p = n;
                        }
                        rx_first[i] = NULL;
                        rx_last[i] = NULL;
                        p = tx_first[i];
                        while (p) {
                                n = p->next;
#if MEM_DEBUG > 0
                                usb_free(p, __LINE__);
#else
                                usb_free(p);
#endif
                                p = n;
                        }
                        tx_first[i] = NULL;
                        tx_last[i] = NULL;
                        usb_rx_byte_count_data[i] = 0;
                        switch (tx_state[i]) {
                          case TX_STATE_EVEN_FREE:
                          case TX_STATE_NONE_FREE_EVEN_FIRST:
                                tx_state[i] = TX_STATE_BOTH_FREE_EVEN_FIRST;
                                break;
                          case TX_STATE_ODD_FREE:
                          case TX_STATE_NONE_FREE_ODD_FIRST:
                                tx_state[i] = TX_STATE_BOTH_FREE_ODD_FIRST;
                                break;
                          default:
                                break;
                        }
                }
#ifndef USB_POOL
                usb_rx_memory_needed = 0;
#endif
                for (i=1; i <= NUM_ENDPOINTS; i++) {
#ifdef USB_POOL
                        usb_rx_memory_needed[i-1] = 0;
#endif
                        epconf = *cfg++;
                        *reg = epconf;
                        reg += 4;
#ifdef AUDIO_INTERFACE
                        if (i == AUDIO_RX_ENDPOINT) {
                                table[index(i, RX, EVEN)].addr = usb_audio_receive_buffer;
                                table[index(i, RX, EVEN)].desc = (AUDIO_RX_SIZE<<16) | BDT_OWN;
                                table[index(i, RX, ODD)].addr = usb_audio_receive_buffer;
                                table[index(i, RX, ODD)].desc = (AUDIO_RX_SIZE<<16) | BDT_OWN;
                        } else
#endif
                        if (epconf & USB_ENDPT_EPRXEN) {
                                usb_packet_t *p;
#ifdef USB_POOL
#if MEM_DEBUG > 0
                                p = usb_malloc(i, __LINE__);
#else
                                p = usb_malloc(i);
#endif
#else
                                p = usb_malloc();
#endif
                                if (p) {
                                        table[index(i, RX, EVEN)].addr = p->buf;
                                        table[index(i, RX, EVEN)].desc = BDT_DESC(64, 0);
                                } else {
                                        table[index(i, RX, EVEN)].addr = 0;
                                        table[index(i, RX, EVEN)].desc = 0;
#ifdef USB_POOL
                                        ++usb_rx_memory_needed[i-1];
                                        // UsbLog("Request %d\r\n", i-1);
#else
                                        ++usb_rx_memory_needed;
#endif
                                }
#ifdef USB_POOL
#if MEM_DEBUG > 0
                                p = usb_malloc(i, __LINE__);
#else
                                p = usb_malloc(i);
#endif
#else
                                p = usb_malloc();
#endif
                                if (p) {
                                        table[index(i, RX, ODD)].addr = p->buf;
                                        table[index(i, RX, ODD)].desc = BDT_DESC(64, 1);
                                } else {
                                        table[index(i, RX, ODD)].addr = 0;
                                        table[index(i, RX, ODD)].desc = 0;
#ifdef USB_POOL
                                        ++usb_rx_memory_needed[i-1];
                                        // UsbLog("Request %d\r\n", i-1);
#else
                                        ++usb_rx_memory_needed;
#endif
                                }
                        }
                        table[index(i, TX, EVEN)].addr = 0;
                        table[index(i, TX, EVEN)].desc = 0;
                        table[index(i, TX, ODD)].addr = 0;
                        table[index(i, TX, ODD)].desc = 0;
#ifdef AUDIO_INTERFACE
                        if (i == AUDIO_SYNC_ENDPOINT) {
                                table[index(i, TX, EVEN)].addr = &usb_audio_sync_feedback;
                                table[index(i, TX, EVEN)].desc = (3<<16) | BDT_OWN;
                        }
#endif
                }
                break;
          case 0x0880: // GET_CONFIGURATION
                reply_buffer[0] = usb_configuration;
                datalen = 1;
                data = reply_buffer;
                break;
          case 0x0080: // GET_STATUS (device)
                reply_buffer[0] = 0;
                reply_buffer[1] = 0;
                datalen = 2;
                data = reply_buffer;
                break;
          case 0x0082: // GET_STATUS (endpoint)
                i = setup.wIndex & 0x7F;
                if (i > NUM_ENDPOINTS) {
                        // TODO: do we need to handle IN vs OUT here?
                        endpoint0_stall();
                        return;
                }
                reply_buffer[0] = 0;
                reply_buffer[1] = 0;
                if (*(uint8_t *)(&USB0_ENDPT0 + i * 4) & 0x02) reply_buffer[0] = 1;
                data = reply_buffer;
                datalen = 2;
                break;
          case 0x0102: // CLEAR_FEATURE (endpoint)
                i = setup.wIndex & 0x7F;
                if (i > NUM_ENDPOINTS || setup.wValue != 0) {
                        // TODO: do we need to handle IN vs OUT here?
                        endpoint0_stall();
                        return;
                }
                (*(uint8_t *)(&USB0_ENDPT0 + i * 4)) &= ~0x02;
                // TODO: do we need to clear the data toggle here?
                break;
          case 0x0302: // SET_FEATURE (endpoint)
                i = setup.wIndex & 0x7F;
                if (i > NUM_ENDPOINTS || setup.wValue != 0) {
                        // TODO: do we need to handle IN vs OUT here?
                        endpoint0_stall();
                        return;
                }
                (*(uint8_t *)(&USB0_ENDPT0 + i * 4)) |= 0x02;
                // TODO: do we need to clear the data toggle here?
                break;
          case 0x0680: // GET_DESCRIPTOR
          case 0x0681:
                //serial_print("desc:");
                //serial_phex16(setup.wValue);
                //serial_print("\n");
                for (list = usb_descriptor_list; 1; list++) {
                        if (list->addr == NULL) break;
                        //if (setup.wValue == list->wValue &&
                        //(setup.wIndex == list->wIndex) || ((setup.wValue >> 8) == 3)) {
                        if (setup.wValue == list->wValue && setup.wIndex == list->wIndex) {
                                data = list->addr;
                                if ((setup.wValue >> 8) == 3) {
                                        // for string descriptors, use the descriptor's
                                        // length field, allowing runtime configured
                                        // length.
                                        datalen = *(list->addr);
                                } else {
                                        datalen = list->length;
                                }
#if 0
                                serial_print("Desc found, ");
                                serial_phex32((uint32_t)data);
                                serial_print(",");
                                serial_phex16(datalen);
                                serial_print(",");
                                serial_phex(data[0]);
                                serial_phex(data[1]);
                                serial_phex(data[2]);
                                serial_phex(data[3]);
                                serial_phex(data[4]);
                                serial_phex(data[5]);
                                serial_print("\n");
#endif
                                goto send;
                        }
                }
                //serial_print("desc: not found\n");
                endpoint0_stall();
                return;
          case 0x2221: // CDC_SET_CONTROL_LINE_STATE
                switch (setup.wIndex) {
#ifdef CDC_STATUS_INTERFACE
                  case CDC_STATUS_INTERFACE:
                        usb_cdc_line_rtsdtr_millis = systick_millis_count;
                        usb_cdc_line_rtsdtr = setup.wValue;
                        break;
#endif
#ifdef CDC2_STATUS_INTERFACE
                  case CDC2_STATUS_INTERFACE:
                        usb_cdc2_line_rtsdtr_millis = systick_millis_count;
                        usb_cdc2_line_rtsdtr = setup.wValue;
                        break;
#endif
#ifdef CDC3_STATUS_INTERFACE
                  case CDC3_STATUS_INTERFACE:
                        usb_cdc3_line_rtsdtr_millis = systick_millis_count;
                        usb_cdc3_line_rtsdtr = setup.wValue;
                        break;
#endif
                }
                //serial_print("set control line state\n");
                break;
#ifdef CDC_STATUS_INTERFACE
          case 0x2321: // CDC_SEND_BREAK
                break;
          case 0x2021: // CDC_SET_LINE_CODING
                //serial_print("set coding, waiting...\n");
                return;
#endif

#if defined(MTP_INTERFACE)
        case 0x64A1: // Cancel Request (PTP spec, 5.2.1, page 8)
                // TODO: required by PTP spec
                endpoint0_stall();
                return;
        case 0x66A1: // Device Reset (PTP spec, 5.2.3, page 10)
                // TODO: required by PTP spec
                endpoint0_stall();
                return;
        case 0x67A1: // Get Device Statis (PTP spec, 5.2.4, page 10)
                // For now, always respond with status ok.
                reply_buffer[0] = 0x4;
                reply_buffer[1] = 0;
                reply_buffer[2] = 0x01;
                reply_buffer[3] = 0x20;
                data = reply_buffer;
                datalen = 4;
                break;
#endif

// TODO: this does not work... why?
#if defined(SEREMU_INTERFACE) || defined(KEYBOARD_INTERFACE)
          case 0x0921: // HID SET_REPORT
                //serial_print(":)\n");
                return;
          case 0x0A21: // HID SET_IDLE
                break;
          // case 0xC940:
#endif

#if defined(AUDIO_INTERFACE)
          case 0x0B01: // SET_INTERFACE (alternate setting)
                if (setup.wIndex == AUDIO_INTERFACE+1) {
                        usb_audio_transmit_setting = setup.wValue;
                        if (usb_audio_transmit_setting > 0) {
                                bdt_t *b = &table[index(AUDIO_TX_ENDPOINT, TX, EVEN)];
                                uint8_t state = tx_state[AUDIO_TX_ENDPOINT-1];
                                if (state) b++;
                                if (!(b->desc & BDT_OWN)) {
                                        memset(usb_audio_transmit_buffer, 0, 176);
                                        b->addr = usb_audio_transmit_buffer;
                                        b->desc = (176 << 16) | BDT_OWN;
                                        tx_state[AUDIO_TX_ENDPOINT-1] = state ^ 1;
                                }
                        }
                } else if (setup.wIndex == AUDIO_INTERFACE+2) {
                        usb_audio_receive_setting = setup.wValue;
                } else {
                        endpoint0_stall();
                        return;
                }
                break;
          case 0x0A81: // GET_INTERFACE (alternate setting)
                datalen = 1;
                data = reply_buffer;
                if (setup.wIndex == AUDIO_INTERFACE+1) {
                        reply_buffer[0] = usb_audio_transmit_setting;
                } else if (setup.wIndex == AUDIO_INTERFACE+2) {
                        reply_buffer[0] = usb_audio_receive_setting;
                } else {
                        endpoint0_stall();
                        return;
                }
                break;
          case 0x0121: // SET FEATURE
          case 0x0221:
          case 0x0321:
          case 0x0421:
                // handle these on the next packet. See usb_audio_set_feature()
                return;
          case 0x81A1: // GET FEATURE
          case 0x82A1:
          case 0x83A1:
          case 0x84A1:
                if (usb_audio_get_feature(&setup, reply_buffer, &datalen)) {
                        data = reply_buffer;
                }
                else {
                        endpoint0_stall();
                        return;
                }
                break;

          case 0x81A2: // GET_CUR (wValue=0, wIndex=interface, wLength=len)
                if (setup.wLength >= 3) {
                        reply_buffer[0] = 44100 & 255;
                        reply_buffer[1] = 44100 >> 8;
                        reply_buffer[2] = 0;
                        datalen = 3;
                        data = reply_buffer;
                } else {
                        endpoint0_stall();
                        return;
                }
                break;
#endif

#if defined(MULTITOUCH_INTERFACE)
          case 0x01A1:
                if (setup.wValue == 0x0300 && setup.wIndex == MULTITOUCH_INTERFACE) {
                        reply_buffer[0] = MULTITOUCH_FINGERS;
                        data = reply_buffer;
                        datalen = 1;
                } else if (setup.wValue == 0x0100 && setup.wIndex == MULTITOUCH_INTERFACE) {
                        memset(reply_buffer, 0, 8);
                        data = reply_buffer;
                        datalen = 8;
                } else {
                        endpoint0_stall();
                        return;
                }
                break;
#endif
#if defined(USB_BLASTER)
            case 0x90C0:
                reply_buffer[0] = blaster_eeprom (2 * setup.wIndex);
                reply_buffer[1] = blaster_eeprom (2 * setup.wIndex + 1);
                datalen = 2;
                data = reply_buffer;
                break;
#endif
          default:
#if defined(USB_BLASTER)
              if ( setup.wRequestAndType & 0x40 )
                  {
                  // Vendor request
                  if ( setup.wRequestAndType & 0x80 )
                      {
                      // Input request
                      reply_buffer[0] = 0x36;
                      reply_buffer[1] = 0x83;
                      datalen = 2;
                      data = reply_buffer;
                      }
                  else
                      {
                      // Output request
                      datalen = 0;
                      data = reply_buffer;
                      }
                  break;
                  }
#endif
                endpoint0_stall();
                return;
        }
        send:
        //serial_print("setup send ");
        //serial_phex32(data);
        //serial_print(",");
        //serial_phex16(datalen);
        //serial_print("\n");

        if (datalen > setup.wLength) datalen = setup.wLength;
        size = datalen;
        if (size > EP0_SIZE) size = EP0_SIZE;
        endpoint0_transmit(data, size);
        data += size;
        datalen -= size;
        if (datalen == 0 && size < EP0_SIZE) return;

        size = datalen;
        if (size > EP0_SIZE) size = EP0_SIZE;
        endpoint0_transmit(data, size);
        data += size;
        datalen -= size;
        if (datalen == 0 && size < EP0_SIZE) return;

        ep0_tx_ptr = data;
        ep0_tx_len = datalen;
}



//A bulk endpoint's toggle sequence is initialized to DATA0 when the endpoint
//experiences any configuration event (configuration events are explained in
//Sections 9.1.1.5 and 9.4.5).

//Configuring a device or changing an alternate setting causes all of the status
//and configuration values associated with endpoints in the affected interfaces
//to be set to their default values. This includes setting the data toggle of
//any endpoint using data toggles to the value DATA0.

//For endpoints using data toggle, regardless of whether an endpoint has the
//Halt feature set, a ClearFeature(ENDPOINT_HALT) request always results in the
//data toggle being reinitialized to DATA0.



// #define stat2bufferdescriptor(stat) (table + ((stat) >> 2))

static void usb_control(uint32_t stat)
{
        bdt_t *b;
        uint32_t pid, size;
        uint8_t *buf;
        const uint8_t *data;

        b = stat2bufferdescriptor(stat);
        pid = BDT_PID(b->desc);
        //count = b->desc >> 16;
        buf = b->addr;
        //serial_print("pid:");
        //serial_phex(pid);
        //serial_print(", count:");
        //serial_phex(count);
        //serial_print("\n");

        switch (pid) {
        case 0x0D: // Setup received from host
                //serial_print("PID=Setup\n");
                //if (count != 8) ; // panic?
                // grab the 8 byte setup info
                setup.word1 = *(uint32_t *)(buf);
                setup.word2 = *(uint32_t *)(buf + 4);

                // give the buffer back
                b->desc = BDT_DESC(EP0_SIZE, DATA1);
                //table[index(0, RX, EVEN)].desc = BDT_DESC(EP0_SIZE, 1);
                //table[index(0, RX, ODD)].desc = BDT_DESC(EP0_SIZE, 1);

                // clear any leftover pending IN transactions
                ep0_tx_ptr = NULL;
                if (ep0_tx_data_toggle) {
                }
                //if (table[index(0, TX, EVEN)].desc & 0x80) {
                        //serial_print("leftover tx even\n");
                //}
                //if (table[index(0, TX, ODD)].desc & 0x80) {
                        //serial_print("leftover tx odd\n");
                //}
                table[index(0, TX, EVEN)].desc = 0;
                table[index(0, TX, ODD)].desc = 0;
                // first IN after Setup is always DATA1
                ep0_tx_data_toggle = 1;

#if 0
                serial_print("bmRequestType:");
                serial_phex(setup.bmRequestType);
                serial_print(", bRequest:");
                serial_phex(setup.bRequest);
                serial_print(", wValue:");
                serial_phex16(setup.wValue);
                serial_print(", wIndex:");
                serial_phex16(setup.wIndex);
                serial_print(", len:");
                serial_phex16(setup.wLength);
                serial_print("\n");
#endif
                // actually "do" the setup request
                usb_setup();
                // unfreeze the USB, now that we're ready
                USB0_CTL = USB_CTL_USBENSOFEN; // clear TXSUSPENDTOKENBUSY bit
                break;
        case 0x01:  // OUT transaction received from host
        case 0x02:
            // UsbLog ("EP0 OUT 0x%04X\r\n", setup.wRequestAndType);
                //serial_print("PID=OUT\n");
                if (setup.wRequestAndType == 0x2021 /*CDC_SET_LINE_CODING*/) {
                        int i;
                        uint32_t *line_coding = NULL;
                        switch (setup.wIndex) {
#ifdef CDC_STATUS_INTERFACE
                          case CDC_STATUS_INTERFACE:
                                line_coding = usb_cdc_line_coding;
                                break;
#endif
#ifdef CDC2_STATUS_INTERFACE
                          case CDC2_STATUS_INTERFACE:
                                line_coding = usb_cdc2_line_coding;
                                break;
#endif
#ifdef CDC3_STATUS_INTERFACE
                          case CDC3_STATUS_INTERFACE:
                                line_coding = usb_cdc3_line_coding;
                                break;
#endif
                        }
                        if (line_coding) {
                                uint8_t *dst = (uint8_t *)line_coding;
                                //serial_print("set line coding ");
                                for (i=0; i<7; i++) {
                                        //serial_phex(*buf);
                                        *dst++ = *buf++;
                                }
                                //serial_phex32(line_coding[0]);
                                //serial_print("\n");
                                if (line_coding[0] == 134) usb_reboot_timer = 15;
                        }
                        endpoint0_transmit(NULL, 0);
                }
#ifdef KEYBOARD_INTERFACE
                if (setup.word1 == 0x02000921 && setup.word2 == ((1<<16)|KEYBOARD_INTERFACE)) {
                        keyboard_leds = buf[0];
                        endpoint0_transmit(NULL, 0);
                }
#endif
#ifdef SEREMU_INTERFACE
                if (setup.word1 == 0x03000921 && setup.word2 == ((4<<16)|SEREMU_INTERFACE)
                  && buf[0] == 0xA9 && buf[1] == 0x45 && buf[2] == 0xC2 && buf[3] == 0x6B) {
                        usb_reboot_timer = 5;
                        endpoint0_transmit(NULL, 0);
                }
#endif
#ifdef AUDIO_INTERFACE
                if (usb_audio_set_feature(&setup, buf)) {
                        endpoint0_transmit(NULL, 0);
                }
#endif
                // give the buffer back
                b->desc = BDT_DESC(EP0_SIZE, DATA1);
                break;

        case 0x09: // IN transaction completed to host
                //serial_print("PID=IN:");
                //serial_phex(stat);
                //serial_print("\n");

                // send remaining data, if any...
                data = ep0_tx_ptr;
                if (data) {
                        size = ep0_tx_len;
                        if (size > EP0_SIZE) size = EP0_SIZE;
                        endpoint0_transmit(data, size);
                        data += size;
                        ep0_tx_len -= size;
                        ep0_tx_ptr = (ep0_tx_len > 0 || size == EP0_SIZE) ? data : NULL;
                }

                if (setup.bRequest == 5 && setup.bmRequestType == 0) {
                        setup.bRequest = 0;
                        //serial_print("set address: ");
                        //serial_phex16(setup.wValue);
                        //serial_print("\n");
                        USB0_ADDR = setup.wValue;
                }

                break;
        default:
            // UsbLog ("PID = 0x%02X\r\n", pid);
                //serial_print("PID=unknown:");
                //serial_phex(pid);
                //serial_print("\n");
            break;
        }
        USB0_CTL = USB_CTL_USBENSOFEN; // clear TXSUSPENDTOKENBUSY bit
}






usb_packet_t *usb_rx(uint32_t endpoint)
{
        usb_packet_t *ret;
        endpoint--;
        if (endpoint >= NUM_ENDPOINTS) return NULL;
        __disable_irq();
        ret = rx_first[endpoint];
        if (ret) {
                rx_first[endpoint] = ret->next;
                usb_rx_byte_count_data[endpoint] -= ret->len;
        }
        __enable_irq();
        //serial_print("rx, epidx=");
        //serial_phex(endpoint);
        //serial_print(", packet=");
        //serial_phex32(ret);
        //serial_print("\n");
        return ret;
}

static uint32_t usb_queue_byte_count(const usb_packet_t *p)
{
        uint32_t count=0;

        __disable_irq();
        for ( ; p; p = p->next) {
                count += p->len;
        }
        __enable_irq();
        return count;
}

// TODO: make this an inline function...
/*
uint32_t usb_rx_byte_count(uint32_t endpoint)
{
        endpoint--;
        if (endpoint >= NUM_ENDPOINTS) return 0;
        return usb_rx_byte_count_data[endpoint];
        //return usb_queue_byte_count(rx_first[endpoint]);
}
*/

uint32_t usb_tx_byte_count(uint32_t endpoint)
{
        endpoint--;
        if (endpoint >= NUM_ENDPOINTS) return 0;
        return usb_queue_byte_count(tx_first[endpoint]);
}

// Discussion about using this function and USB transmit latency
// https://forum.pjrc.com/threads/58663?p=223513&viewfull=1#post223513
//
uint32_t usb_tx_packet_count(uint32_t endpoint)
{
        const usb_packet_t *p;
        uint32_t count=0;

        endpoint--;
        if (endpoint >= NUM_ENDPOINTS) return 0;
        __disable_irq();
        for (p = tx_first[endpoint]; p; p = p->next) count++;
        __enable_irq();
        return count;
}


// Called from usb_free, but only when usb_rx_memory_needed > 0, indicating
// receive endpoints are starving for memory.  The intention is to give
// endpoints needing receive memory priority over the user's code, which is
// likely calling usb_malloc to obtain memory for transmitting.  When the
// user is creating data very quickly, their consumption could starve reception
// without this prioritization.  The packet buffer (input) is assigned to the
// first endpoint needing memory.
//
void usb_rx_memory(usb_packet_t *packet)
{
        unsigned int i;
        const uint8_t *cfg;

        cfg = usb_endpoint_config_table;
        //serial_print("rx_mem:");
        __disable_irq();
#ifdef USB_POOL
        i = packet->iPool + 1;
        cfg += i - 1;
                {
#else
        for (i=1; i <= NUM_ENDPOINTS; i++) {
#endif
#ifdef AUDIO_INTERFACE
                if (i == AUDIO_RX_ENDPOINT) continue;
#endif
                if (*cfg++ & USB_ENDPT_EPRXEN) {
                        if (table[index(i, RX, EVEN)].desc == 0) {
                                table[index(i, RX, EVEN)].addr = packet->buf;
                                table[index(i, RX, EVEN)].desc = BDT_DESC(64, 0);
#ifdef USB_POOL
                                --usb_rx_memory_needed[i-1];
#else
                                --usb_rx_memory_needed;
#endif
                                __enable_irq();
                                //serial_phex(i);
                                //serial_print(",even\n");
                                return;
                        }
                        if (table[index(i, RX, ODD)].desc == 0) {
                                table[index(i, RX, ODD)].addr = packet->buf;
                                table[index(i, RX, ODD)].desc = BDT_DESC(64, 1);
#ifdef USB_POOL
                                --usb_rx_memory_needed[i-1];
#else
                                --usb_rx_memory_needed;
#endif
                                __enable_irq();
                                //serial_phex(i);
                                //serial_print(",odd\n");
                                return;
                        }
                }
        }
        __enable_irq();
        // we should never reach this point.  If we get here, it means
        // usb_rx_memory_needed was set greater than zero, but no memory
        // was actually needed.
#ifdef USB_POOL
        usb_rx_memory_needed[i-1] = 0;
#else
        usb_rx_memory_needed = 0;
#endif
#if MEM_DEBUG > 0
        usb_free(packet, __LINE__);
#else
        usb_free(packet);
#endif
        return;
}

//#define index(endpoint, tx, odd) (((endpoint) << 2) | ((tx) << 1) | (odd))
//#define stat2bufferdescriptor(stat) (table + ((stat) >> 2))

void usb_tx(uint32_t endpoint, usb_packet_t *packet)
{
        bdt_t *b = &table[index(endpoint, TX, EVEN)];
        uint8_t next;

        // if (packet->len == 0) UsbLog ("Zero length packet.\r\n");

        endpoint--;
        if (endpoint >= NUM_ENDPOINTS) return;
        __disable_irq();
        //serial_print("txstate=");
        //serial_phex(tx_state[endpoint]);
        //serial_print("\n");
        switch (tx_state[endpoint]) {
          case TX_STATE_BOTH_FREE_EVEN_FIRST:
                next = TX_STATE_ODD_FREE;
                break;
          case TX_STATE_BOTH_FREE_ODD_FIRST:
                b++;
                next = TX_STATE_EVEN_FREE;
                break;
          case TX_STATE_EVEN_FREE:
                next = TX_STATE_NONE_FREE_ODD_FIRST;
                break;
          case TX_STATE_ODD_FREE:
                b++;
                next = TX_STATE_NONE_FREE_EVEN_FIRST;
                break;
          default:
                if (tx_first[endpoint] == NULL) {
                        tx_first[endpoint] = packet;
                } else {
                        tx_last[endpoint]->next = packet;
                }
                tx_last[endpoint] = packet;
                __enable_irq();
                return;
        }
        tx_state[endpoint] = next;
        b->addr = packet->buf;
        b->desc = BDT_DESC(packet->len, ((uint32_t)b & 8) ? DATA1 : DATA0);
        __enable_irq();
}

void usb_tx_isochronous(uint32_t endpoint, void *data, uint32_t len)
{
        bdt_t *b = &table[index(endpoint, TX, EVEN)];
        uint8_t next, state;

        endpoint--;
        if (endpoint >= NUM_ENDPOINTS) return;
        __disable_irq();
        state = tx_state[endpoint];
        if (state == 0) {
                next = 1;
        } else {
                b++;
                next = 0;
        }
        tx_state[endpoint] = next;
        b->addr = data;
        b->desc = (len << 16) | BDT_OWN;
        __enable_irq();
}




void _reboot_Teensyduino_(void)
{
        // TODO: initialize R0 with a code....
        __asm__ volatile("bkpt");
        __builtin_unreachable();
}



void usb_isr(void)
{
        uint8_t status, stat, t;

        //serial_print("isr");
        //status = USB0_ISTAT;
        //serial_phex(status);
        //serial_print("\n");
        restart:
        status = USB0_ISTAT;    // Interrupt status register - 46.4.9 in hardware manual

        if ((status & USB_ISTAT_SOFTOK /* 04 */ )) {    // USB Start of Frame received
                if (usb_configuration) {    // Non-zero if a configuration has been set
                        t = usb_reboot_timer;
                        if (t) {
                                usb_reboot_timer = --t;
                                if (!t) _reboot_Teensyduino_();
                        }
#ifdef CDC_DATA_INTERFACE
                        t = usb_cdc_transmit_flush_timer;
                        if (t) {
                                usb_cdc_transmit_flush_timer = --t;
                                if (t == 0) usb_serial_flush_callback();
                        }
#endif
#ifdef CDC2_DATA_INTERFACE
                        t = usb_cdc2_transmit_flush_timer;
                        if (t) {
                                usb_cdc2_transmit_flush_timer = --t;
                                if (t == 0) usb_serial2_flush_callback();
                        }
#endif
#ifdef CDC3_DATA_INTERFACE
                        t = usb_cdc3_transmit_flush_timer;
                        if (t) {
                                usb_cdc3_transmit_flush_timer = --t;
                                if (t == 0) usb_serial3_flush_callback();
                        }
#endif
#ifdef SEREMU_INTERFACE
                        t = usb_seremu_transmit_flush_timer;
                        if (t) {
                                usb_seremu_transmit_flush_timer = --t;
                                if (t == 0) usb_seremu_flush_callback();
                        }
#endif
#ifdef MIDI_INTERFACE
                        usb_midi_flush_output();
#endif
#ifdef FLIGHTSIM_INTERFACE
                        usb_flightsim_flush_callback();
#endif
#ifdef MULTITOUCH_INTERFACE
                        usb_touchscreen_update_callback();
#endif
#ifdef USB_BLASTER
                        blaster_flush ();
#endif
                }
                USB0_ISTAT = USB_ISTAT_SOFTOK;  // Clear interrupt by writing back flag
        }

        if ((status & USB_ISTAT_TOKDNE /* 08 */ )) {    // Finished processing USB token
                uint8_t endpoint;
                stat = USB0_STAT;   // Status register - 46.4.13 in hardware manual
                //serial_print("token: ep=");
                //serial_phex(stat >> 4);
                //serial_print(stat & 0x08 ? ",tx" : ",rx");
                //serial_print(stat & 0x04 ? ",odd\n" : ",even\n");
                endpoint = stat >> 4;
                if (endpoint == 0) {
                        usb_control(stat);
                } else {
                        bdt_t *b = stat2bufferdescriptor(stat);     // Get corresponding buffer descriptor
                        // Obtain current usb_packet_t address from buffer descriptor:
                        // Better option might have been to put buf[] at the top of the
                        // usb_packet_t structure, so that the buf[] and the structure
                        // have the same address.
                        usb_packet_t *packet = (usb_packet_t *)((uint8_t *)(b->addr)
                            - offsetof(usb_packet_t, buf));
#if 0
                        serial_print("ep:");
                        serial_phex(endpoint);
                        serial_print(", pid:");
                        serial_phex(BDT_PID(b->desc));
                        serial_print(((uint32_t)b & 8) ? ", odd" : ", even");
                        serial_print(", count:");
                        serial_phex(b->desc >> 16);
                        serial_print("\n");
#endif
                        endpoint--;     // endpoint is index to zero-based arrays

#ifdef AUDIO_INTERFACE
                        if ((endpoint == AUDIO_TX_ENDPOINT-1) && (stat & 0x08)) {
                                unsigned int len;
                                len = usb_audio_transmit_callback();
                                if (len > 0) {
                                        b = (bdt_t *)((uint32_t)b ^ 8);
                                        b->addr = usb_audio_transmit_buffer;
                                        b->desc = (len << 16) | BDT_OWN;
                                        tx_state[endpoint] ^= 1;
                                }
                        } else if ((endpoint == AUDIO_RX_ENDPOINT-1) && !(stat & 0x08)) {
                                usb_audio_receive_callback(b->desc >> 16);
                                b->addr = usb_audio_receive_buffer;
                                b->desc = (AUDIO_RX_SIZE << 16) | BDT_OWN;
                        } else if ((endpoint == AUDIO_SYNC_ENDPOINT-1) && (stat & 0x08)) {
                                b = (bdt_t *)((uint32_t)b ^ 8);
                                b->addr = &usb_audio_sync_feedback;
                                b->desc = (3 << 16) | BDT_OWN;
                                tx_state[endpoint] ^= 1;
                        } else
#endif
                        if (stat & 0x08) { // transmit
#if MEM_DEBUG > 0
                                usb_free(packet, __LINE__);   // Free the just transmitted packet
#else
                                usb_free(packet);   // Free the just transmitted packet
#endif
                                packet = tx_first[endpoint];    // Get the next queued packet
                                if (packet) {   // If another packet
                                        //serial_print("tx packet\n");
                                        tx_first[endpoint] = packet->next;  // Remove it from the queue
                                        b->addr = packet->buf;  // And link it to the buffer descriptor
                                        // Update which BDs are in use
                                        switch (tx_state[endpoint]) {
                                          case TX_STATE_BOTH_FREE_EVEN_FIRST:
                                                tx_state[endpoint] = TX_STATE_ODD_FREE;
                                                break;
                                          case TX_STATE_BOTH_FREE_ODD_FIRST:
                                                tx_state[endpoint] = TX_STATE_EVEN_FREE;
                                                break;
                                          case TX_STATE_EVEN_FREE:
                                                tx_state[endpoint] = TX_STATE_NONE_FREE_ODD_FIRST;
                                                break;
                                          case TX_STATE_ODD_FREE:
                                                tx_state[endpoint] = TX_STATE_NONE_FREE_EVEN_FIRST;
                                                break;
                                          default:
                                                break;
                                        }
                                        // Set the BD for transmission
                                        b->desc = BDT_DESC(packet->len,
                                                ((uint32_t)b & 8) ? DATA1 : DATA0);
                                } else {
                                        //serial_print("tx no packet\n");
                                        // Update which BDs are in use
                                        switch (tx_state[endpoint]) {
                                          case TX_STATE_BOTH_FREE_EVEN_FIRST:
                                          case TX_STATE_BOTH_FREE_ODD_FIRST:
                                                break;
                                          case TX_STATE_EVEN_FREE:
                                                tx_state[endpoint] = TX_STATE_BOTH_FREE_EVEN_FIRST;
                                                break;
                                          case TX_STATE_ODD_FREE:
                                                tx_state[endpoint] = TX_STATE_BOTH_FREE_ODD_FIRST;
                                                break;
                                          default:
                                                tx_state[endpoint] = ((uint32_t)b & 8) ?
                                                  TX_STATE_ODD_FREE : TX_STATE_EVEN_FREE;
                                                break;
                                        }
                                // Does not update the BD. OWN flag remains clear.
                                // Assume that this results in a NAK if a request to
                                // transmit this BD.
                                }
                        } else { // receive
                                packet->len = b->desc >> 16;    // Get received length from BD
                                if (packet->len > 0) {  // Data received - Add packet to received queue
                                        packet->index = 0;
                                        packet->next = NULL;
                                        if (rx_first[endpoint] == NULL) {
                                                //serial_print("rx 1st, epidx=");
                                                //serial_phex(endpoint);
                                                //serial_print(", packet=");
                                                //serial_phex32((uint32_t)packet);
                                                //serial_print("\n");
                                                rx_first[endpoint] = packet;
                                        } else {
                                                //serial_print("rx Nth, epidx=");
                                                //serial_phex(endpoint);
                                                //serial_print(", packet=");
                                                //serial_phex32((uint32_t)packet);
                                                //serial_print("\n");
                                                rx_last[endpoint]->next = packet;
                                        }
                                        rx_last[endpoint] = packet;
                                        usb_rx_byte_count_data[endpoint] += packet->len;
                                        // TODO: implement a per-endpoint maximum # of allocated
                                        // packets, so a flood of incoming data on 1 endpoint
                                        // doesn't starve the others if the user isn't reading
                                        // it regularly
#ifdef USB_POOL
#if MEM_DEBUG > 0
                                        packet = usb_malloc(endpoint + 1, __LINE__);
#else
                                        packet = usb_malloc(endpoint + 1);
#endif
#else
                                        packet = usb_malloc();
#endif
                                        if (packet) {   // Link new packet to BD
                                                // UsbLog ("Allocate %p\r\n", packet);
                                                b->addr = packet->buf;
                                                b->desc = BDT_DESC(64,
                                                        ((uint32_t)b & 8) ? DATA1 : DATA0);
                                        } else {
                                                //serial_print("starving ");
                                                //serial_phex(endpoint + 1);
                                                b->desc = 0;    // OWN flag not set. NAK / Stall if used?
#ifdef USB_POOL
                                                ++usb_rx_memory_needed[endpoint];
                                                // UsbLog("Request %d\r\n", endpoint);
#else
                                                ++usb_rx_memory_needed;
#endif
                                        }
                                } else {    // No data - reuse the current packet
                                        // UsbLog ("Empty 0x%04X\r\n", b->desc);
                                        b->desc = BDT_DESC(64, ((uint32_t)b & 8) ? DATA1 : DATA0);
                                }
                        }

                }
                USB0_ISTAT = USB_ISTAT_TOKDNE;  // Reset the interrupt
                goto restart;   // There may be another interrupt
        }



        if (status & USB_ISTAT_USBRST /* 01 */ ) {
                //serial_print("reset\n");
                // UsbLog ("Reset\r\n");

                // initialize BDT toggle bits
                USB0_CTL = USB_CTL_ODDRST;  // Hardware 46.4.14
                ep0_tx_bdt_bank = 0;

                // set up buffers to receive Setup and OUT packets
                table[index(0, RX, EVEN)].desc = BDT_DESC(EP0_SIZE, 0);
                table[index(0, RX, EVEN)].addr = ep0_rx0_buf;
                table[index(0, RX, ODD)].desc = BDT_DESC(EP0_SIZE, 0);  // Why is this DATA0?
                table[index(0, RX, ODD)].addr = ep0_rx1_buf;
                table[index(0, TX, EVEN)].desc = 0;
                table[index(0, TX, ODD)].desc = 0;

                // activate endpoint 0 - Tx, Rx & Handshake - 46.4.23
                USB0_ENDPT0 = USB_ENDPT_EPRXEN | USB_ENDPT_EPTXEN | USB_ENDPT_EPHSHK;

                // clear all ending interrupts - Writing one bits clears interrupt
                USB0_ERRSTAT = 0xFF;    // Hardware 46.4.11
                USB0_ISTAT = 0xFF;      // Hardware 46.4.9

                // set the address to zero during enumeration
                USB0_ADDR = 0;          // Hardware 46.4.15

                // enable other interrupts
                USB0_ERREN = 0xFF;                  // Hardware 46.4.12
                USB0_INTEN = USB_INTEN_TOKDNEEN |   // Hardware 46.4.10
                        USB_INTEN_SOFTOKEN |
                        USB_INTEN_STALLEN |
                        USB_INTEN_ERROREN |
                        USB_INTEN_USBRSTEN |
                        USB_INTEN_SLEEPEN;

                // is this necessary?
                USB0_CTL = USB_CTL_USBENSOFEN;      // Hardware 46.4.14
                return;
        }


        if ((status & USB_ISTAT_STALL /* 80 */ )) { // Clear interrupt and reenable endpoint
                //serial_print("stall:\n");
                // UsbLog ("Stall\r\n");
                USB0_ENDPT0 = USB_ENDPT_EPRXEN | USB_ENDPT_EPTXEN | USB_ENDPT_EPHSHK;
                USB0_ISTAT = USB_ISTAT_STALL;
        }
        if ((status & USB_ISTAT_ERROR /* 02 */ )) { // Clear interrupr
                uint8_t err = USB0_ERRSTAT;
                USB0_ERRSTAT = err;
                // UsbLog ("Error 0x%02X\r\n", err);
                //serial_print("err:");
                //serial_phex(err);
                //serial_print("\n");
                USB0_ISTAT = USB_ISTAT_ERROR;
        }

        if ((status & USB_ISTAT_SLEEP /* 10 */ )) { // Clear interrupt
                //serial_print("sleep\n");
                // UsbLog ("Sleep\r\n");
                USB0_ISTAT = USB_ISTAT_SLEEP;
        }

}



void usb_init(void)
{
        int i;

        //serial_begin(BAUD2DIV(115200));
        //serial_print("usb_init\n");

        usb_init_serialnumber();

#ifdef USB_POOL
        if ( ! usb_mem_init () ) return;
#endif

        // This was for (i=0; i <= NUM_ENDPOINTS*4; i++)
        // which left the last three entries undefined
        for (i=0; i < (NUM_ENDPOINTS+1)*4; i++) {
                table[i].desc = 0;
                table[i].addr = 0;
        }

        // this basically follows the flowchart in the Kinetis
        // Quick Reference User Guide, Rev. 1, 03/2012, page 141

        // assume 48 MHz clock already running
        // SIM - enable clock
        SIM_SCGC4 |= SIM_SCGC4_USBOTG;
#ifdef HAS_KINETIS_MPU
        MPU_RGDAAC0 |= 0x03000000;
#endif
#if F_CPU == 180000000 || F_CPU == 216000000 || F_CPU == 256000000
        // if using IRC48M, turn on the USB clock recovery hardware
        USB0_CLK_RECOVER_IRC_EN = USB_CLK_RECOVER_IRC_EN_IRC_EN | USB_CLK_RECOVER_IRC_EN_REG_EN;
        USB0_CLK_RECOVER_CTRL = USB_CLK_RECOVER_CTRL_CLOCK_RECOVER_EN |
                USB_CLK_RECOVER_CTRL_RESTART_IFRTRIM_EN;
#endif
        // reset USB module
        //USB0_USBTRC0 = USB_USBTRC_USBRESET;
        //while ((USB0_USBTRC0 & USB_USBTRC_USBRESET) != 0) ; // wait for reset to end

        // set desc table base addr
        USB0_BDTPAGE1 = ((uint32_t)table) >> 8;
        USB0_BDTPAGE2 = ((uint32_t)table) >> 16;
        USB0_BDTPAGE3 = ((uint32_t)table) >> 24;

        // clear all ISR flags
        USB0_ISTAT = 0xFF;
        USB0_ERRSTAT = 0xFF;
        USB0_OTGISTAT = 0xFF;

        //USB0_USBTRC0 |= 0x40; // undocumented bit

        // enable USB
        USB0_CTL = USB_CTL_USBENSOFEN;
        USB0_USBCTRL = 0;

        // enable reset interrupt
        USB0_INTEN = USB_INTEN_USBRSTEN;

        // enable interrupt in NVIC...
        NVIC_SET_PRIORITY(IRQ_USBOTG, 112);
        NVIC_ENABLE_IRQ(IRQ_USBOTG);

        // enable d+ pullup
        USB0_CONTROL = USB_CONTROL_DPPULLUPNONOTG;
}


#else // F_CPU < 20 MHz && defined(NUM_ENDPOINTS)

void usb_init(void)
{
}

#endif // F_CPU >= 20 MHz && defined(NUM_ENDPOINTS)
