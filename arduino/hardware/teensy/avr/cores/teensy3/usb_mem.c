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
 */

#include "usb_dev.h"
#if F_CPU >= 20000000 && defined(NUM_ENDPOINTS)

#include "kinetis.h"
#include "usb_mem.h"

__attribute__ ((section(".usbbuffers"), used))
unsigned char usb_buffer_memory[NUM_USB_BUFFERS * sizeof(usb_packet_t)];

//#if MEM_DEBUG > 0
#include "HardwareSerial.h"
#include <stdarg.h>
#include <string.h>
#include <stdio.h>

void UsbLog (const char *psFmt,...)
    {
    char sLine[256];
    va_list va;
    va_start (va, psFmt);
    vsprintf (sLine, psFmt, va);
    va_end (va);
    serial2_write (sLine, strlen (sLine));
    }

#if MEM_DEBUG > 0
#define NEVT    20
static struct
    {
    char            type;
    usb_packet_t    *p;
    int             iPool;
    int             iLine;
    }
    usb_evt[NEVT];
static int nEvt = 0;

#if MEM_DEBUG > 1
static int bNull[NUM_ENDPOINTS];
#endif

const usb_packet_t * usb_mem_base(void)
    {
    return (usb_packet_t *) usb_buffer_memory;
    }
#endif  // MEM_DEBUG > 0

#ifdef USB_POOL
static int pool_size[] = USB_POOL;
static usb_packet_t *pool_ptr[NUM_ENDPOINTS];

int usb_mem_init(void)
    {
    int nBuf = NUM_USB_BUFFERS;
    usb_packet_t *ppkt = (usb_packet_t *) usb_buffer_memory;
    if (sizeof (pool_size) / sizeof (int) < NUM_ENDPOINTS)
        {
#if MEM_DEBUG > 0
        UsbLog ("Insufficient pools defined\r\n");
#endif
        return 0;
        }
    for (int iPool = 0; iPool < NUM_ENDPOINTS; ++iPool)
        {
#if MEM_DEBUG > 1
        UsbLog ("Buffer pool %d\r\n", iPool+1);
#endif
        pool_ptr[iPool] = NULL;
        for (int i = 0; i < pool_size[iPool]; ++i)
            {
            if (nBuf == 0)
                {
#if MEM_DEBUG > 0
                UsbLog ("Insufficient USB buffers.\r\n");
#endif
                return 0;
                }
            ppkt->next = pool_ptr[iPool];
            ppkt->iPool = iPool;
            pool_ptr[iPool] = ppkt;
#if MEM_DEBUG > 1
            UsbLog ("ppkt = %p, next = %p\r\n", ppkt, ppkt->next);
#endif
            ++ppkt;
            --nBuf;
            }
#if MEM_DEBUG > 1
        UsbLog ("pool_ptr[%d] = %p\r\n", iPool, pool_ptr[iPool]);
#endif
        }
#if MEM_DEBUG > 1
    UsbLog ("USB buffer pools created.\r\n");
#endif
    return 1;
    }

#if MEM_DEBUG > 0
void usb_mem_show(void)
    {
    for (int i = 0; i < nEvt; ++i) UsbLog ("%c EP%d packet = %p (%d) line %d\r\n",
        usb_evt[i].type, usb_evt[i].iPool+1, usb_evt[i].p,
        usb_evt[i].p - (usb_packet_t *)usb_buffer_memory, usb_evt[i].iLine);
    for (int iPool = 0; iPool < NUM_ENDPOINTS; ++iPool)
        {
        UsbLog("Pool %d:", iPool+1);
        usb_packet_t *p =pool_ptr[iPool];
        while (p != NULL)
            {
            UsbLog(" %d", p - (usb_packet_t *)usb_buffer_memory);
            p = p->next;
            }
        UsbLog("\r\n");
        }
    usb_queues();
    nEvt = 0;
    }
#endif  // MEM_DEBUG > 0

#if MEM_DEBUG > 0
usb_packet_t * usb_malloc(int iEP, int iLine)
#else
usb_packet_t * usb_malloc(int iEP)
#endif
    {
    int iPool = iEP - 1;
    if ((iEP <= 0) || (iEP > NUM_ENDPOINTS))
        {
#if MEM_DEBUG > 0
        if ( nEvt < NEVT )
            {
            usb_evt[nEvt].type = 'D';
            usb_evt[nEvt].p = NULL;
            usb_evt[nEvt].iPool = iPool;
            usb_evt[nEvt].iLine = iLine;
            ++nEvt;
            }
#endif
        return NULL;
        }
	__disable_irq();
    usb_packet_t *ppkt = pool_ptr[iPool];
    if ( ppkt != NULL )
        {
        pool_ptr[iPool] = ppkt->next;
        ppkt->len = 0;
        ppkt->index = 0;
        ppkt->next = NULL;
        }
#if MEM_DEBUG > 0
    if ( nEvt < NEVT )
        {
        usb_evt[nEvt].type = 'A';
        usb_evt[nEvt].p = ppkt;
        usb_evt[nEvt].iPool = iPool;
        usb_evt[nEvt].iLine = iLine;
        ++nEvt;
        }
#endif
	__enable_irq();
#if MEM_DEBUG > 1
    if ((ppkt != NULL) || (bNull[iEP]))
        {
        UsbLog ("usb_malloc (%d) = %p, pool_ptr[%d] = %p\r\n", iEP, ppkt, iPool, pool_ptr[iPool]);
        bNull[iEP] = ppkt != NULL;
        }
#endif
    return ppkt;
    }

// for the receive endpoints to request memory
extern uint8_t usb_rx_memory_needed[NUM_ENDPOINTS];
extern void usb_rx_memory(usb_packet_t *packet);

#if MEM_DEBUG > 0
void usb_free(usb_packet_t *ppkt, int iLine)
#else
void usb_free(usb_packet_t *ppkt)
#endif
    {
    int iPool = ppkt->iPool;
    if ((iPool < 0) || (iPool >= NUM_ENDPOINTS))
        {
#if MEM_DEBUG > 0
        if ( nEvt < NEVT )
            {
            usb_evt[nEvt].type = 'C';
            usb_evt[nEvt].p = ppkt;
            usb_evt[nEvt].iPool = iPool;
            usb_evt[nEvt].iLine = iLine;
            ++nEvt;
            }
#endif
        return;
        }
    // UsbLog ("Mem needed: %d %d\r\n", usb_rx_memory_needed[0], usb_rx_memory_needed[1]);
	// if the endpoint is starving for memory to receive
	// packets, give this memory to them immediately!
    // Essential, as endpoint does not retry memory allocation if initially failed.
	if (usb_rx_memory_needed[iPool] && usb_configuration) {
        // UsbLog ("Assign packet\r\n");
		usb_rx_memory(ppkt);
		return;
	}
    
	__disable_irq();
	unsigned int n = ((uint8_t *)ppkt - usb_buffer_memory) / sizeof(usb_packet_t);
	if (n >= NUM_USB_BUFFERS)
        {
#if MEM_DEBUG > 0
        if ( nEvt < NEVT )
            {
            usb_evt[nEvt].type = 'B';
            usb_evt[nEvt].p = ppkt;
            usb_evt[nEvt].iPool = 0;
            usb_evt[nEvt].iLine = iLine;
            ++nEvt;
            }
#endif
        return;
        }
#if MEM_DEBUG > 0
    if ( nEvt < NEVT )
        {
        usb_evt[nEvt].type = 'F';
        usb_evt[nEvt].p = ppkt;
        usb_evt[nEvt].iPool = ppkt->iPool;
        usb_evt[nEvt].iLine = iLine;
        ++nEvt;
        }
#endif
    ppkt->next = pool_ptr[iPool];
    pool_ptr[iPool] = ppkt;
	__enable_irq();
#if MEM_DEBUG > 1
    UsbLog ("usb_free (%p), iPool = %d, next = %p\r\n", ppkt, iPool, ppkt->next);
#endif
    }
#else   // USB_POOL not defined

// use bitmask and CLZ instruction to implement fast free list
// http://www.archivum.info/gnu.gcc.help/2006-08/00148/Re-GCC-Inline-Assembly.html
// http://gcc.gnu.org/ml/gcc/2012-06/msg00015.html
// __builtin_clz()

static uint32_t usb_buffer_available = 0xFFFFFFFF;

#if MEM_DEBUG > 0
void usb_mem_show(void)
    {
    UsbLog("usb_buffer_available = 0x%08X\r\n", usb_buffer_available);
    usb_queues();
    }
#endif

usb_packet_t * usb_malloc(void)
{
	unsigned int n, avail;
	uint8_t *p;

	__disable_irq();
	avail = usb_buffer_available;
	n = __builtin_clz(avail); // clz = count leading zeros
	if (n >= NUM_USB_BUFFERS) {
		__enable_irq();
		return NULL;
	}
	//serial_print("malloc:");
	//serial_phex(n);
	//serial_print("\r\n");

	usb_buffer_available = avail & ~(0x80000000 >> n);
	__enable_irq();
	p = usb_buffer_memory + (n * sizeof(usb_packet_t));
	//serial_print("malloc:");
	//serial_phex32((int)p);
	//serial_print("\r\n");
	*(uint32_t *)p = 0;
	*(uint32_t *)(p + 4) = 0;
	return (usb_packet_t *)p;
}

// for the receive endpoints to request memory
extern uint8_t usb_rx_memory_needed;
extern void usb_rx_memory(usb_packet_t *packet);

void usb_free(usb_packet_t *p)
{
	unsigned int n, mask;

	//serial_print("free:");
	n = ((uint8_t *)p - usb_buffer_memory) / sizeof(usb_packet_t);
	if (n >= NUM_USB_BUFFERS) return;
	//serial_phex(n);
	//serial_print("\r\n");

	// if any endpoints are starving for memory to receive
	// packets, give this memory to them immediately!
    // Essential, as endpoint does not retry memory allocation if initially failed.
	if (usb_rx_memory_needed && usb_configuration) {
		//serial_print("give to rx:");
		//serial_phex32((int)p);
		//serial_print("\r\n");
		usb_rx_memory(p);
		return;
	}

	mask = (0x80000000 >> n);
	__disable_irq();
	usb_buffer_available |= mask;
	__enable_irq();

	//serial_print("free:");
	//serial_phex32((int)p);
	//serial_print("\r\n");
}
#endif  // defined USB_POOL

#endif // F_CPU >= 20 MHz && defined(NUM_ENDPOINTS)
