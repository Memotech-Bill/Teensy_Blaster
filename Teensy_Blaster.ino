#include "usb_dev.h"
#include "eeprom.h"
#include "HardwareSerial.h"

#define DEBUG       0
#define SHOW_LED    1

// GPIO Pins
#define PIN_TCK     0
#define PIN_TMS     1
#define PIN_NCE     2
#define PIN_NCS     3
#define PIN_TDI     4
#define PIN_TDO     5
#define PIN_ASO     6
#define PIN_CNT     7
#define PIN_LED     13

// Blaster input bits
#define BIT_TDO     0x01
#define BIT_ASO     0x02

// Blaster output bits
#define BIT_TCK     0x01
#define BIT_TMS     0x02
#define BIT_NCE     0x04
#define BIT_NCS     0x08
#define BIT_TDI     0x10
#define BIT_ACT     0x20
#define BITS_PORT   0x1F

// Protocol bits
#define BIT_RD      0x40
#define BIT_SEQ     0x80
#define BITS_CNT    0x3F

#define SEND_INT    10      // Time (miliseconds) between empty packets

#if MEM_DEBUG > 0
static const usb_packet_t *pbase = NULL;
#endif
static usb_packet_t *ptx = NULL;
static uint8_t uPort = BIT_TMS | BIT_TDI | BIT_NCE | BIT_NCS;
static uint8_t uRead = 0;
static int nSeq = 0;
static int bRead = 0;
static uint32_t tNext = 0;
#if DEBUG > 0
int tShow;
#endif

#if DEBUG > 1
static const char *psBits[] = {"TCK", "TMS", "NCE", "NCS", "TDI", "ACT", "RD ", "SEQ"};
#endif

void setup()
{
  // Hardware serial for diagnostics
//#if (DEBUG > 0) || (MEM_DEBUG > 0)
  Serial2.begin (115200, SERIAL_8N1);
//#endif
  
  // Configure JTAG Pins
  pinMode (PIN_TCK, OUTPUT);
  digitalWrite (PIN_TCK, LOW);
  pinMode (PIN_TMS, OUTPUT);
  digitalWrite (PIN_TMS, HIGH);
  pinMode (PIN_TDI, OUTPUT);
  digitalWrite (PIN_TDI, HIGH);
  pinMode (PIN_NCE, OUTPUT);
  digitalWrite (PIN_NCE, HIGH);
  pinMode (PIN_NCS, OUTPUT);
  digitalWrite (PIN_NCS, HIGH);
#if SHOW_LED
  pinMode (PIN_LED, OUTPUT);
  digitalWrite (PIN_LED, LOW);
#endif
  pinMode (PIN_TDO, INPUT_PULLUP);
  pinMode (PIN_ASO, INPUT_PULLUP);

  // Initialise USB
#if (DEBUG > 0) || (MEM_DEBUG > 0)
  Serial2.printf ("Initialise USB\r\n");
#endif
  usb_init ();
#if MEM_DEBUG > 0
  usb_mem_show();
  pbase = usb_mem_base ();
  tShow = millis() + 10000;
#endif

  // Initialise empty packet timeout
  tNext = millis () + SEND_INT;
}

void JTAG_WR (uint8_t uPins)
{
  digitalWrite (PIN_TMS, ( uPins & BIT_TMS ) ? HIGH : LOW);
  digitalWrite (PIN_TDI, ( uPins & BIT_TDI ) ? HIGH : LOW);
  digitalWrite (PIN_NCE, ( uPins & BIT_NCE ) ? HIGH : LOW);
  digitalWrite (PIN_NCS, ( uPins & BIT_NCS ) ? HIGH : LOW);
  digitalWrite (PIN_TCK, ( uPins & BIT_TCK ) ? HIGH : LOW);
}

uint8_t JTAG_RD (void)
{
  uint8_t uPins = 0;
  if ( digitalRead (PIN_TDO) ) uPins |= BIT_TDO;
  if ( digitalRead (PIN_ASO) ) uPins |= BIT_ASO;
  return uPins;
}

uint8_t blaster_eeprom (uint16_t addr)
{
  return bEEPROM[addr];
}

void blaster_flush (void)
{
}

void blaster_alloc (void)
{
  if (ptx == NULL)
  {
#if MEM_DEBUG > 0
    Serial2.printf ("Request allocation: ");
#endif
#ifdef USB_POOL
#if MEM_DEBUG
    ptx = usb_malloc (BLASTER_TX_EP, -1);
#else
    ptx = usb_malloc (BLASTER_TX_EP);
#endif
#else
    ptx = usb_malloc ();
#endif
    while (ptx == NULL)
    {
      yield ();
#ifdef USB_POOL
#if MEM_DEBUG
      ptx = usb_malloc (BLASTER_TX_EP, -2);
#else
      ptx = usb_malloc (BLASTER_TX_EP);
#endif
#else
      ptx = usb_malloc ();
#endif
    }
#if MEM_DEBUG > 0
    Serial2.printf ("ptx = %p (%d)\r\n", ptx, ptx - pbase);
    usb_mem_show();
#endif
    ptx->buf[0] = 0x31;
    ptx->buf[1] = 0x60;
    ptx->len = 2;
  }
}

void blaster_tx (void)
{
  if (ptx != NULL)
  {
#if DEBUG > 0
    Serial2.printf ("Send:");
    for (int i = 0; i < ptx->len; ++i ) Serial2.printf (" %02X", ptx->buf[i]);
    Serial2.printf ("\r\n");
#endif
    usb_tx (BLASTER_TX_EP, ptx);
    ptx = NULL;
  }
}

void blaster_send (uint8_t u)
{
#if DEBUG > 1
  Serial2.printf ("Queue: %02X, nTxIn = %d, nTxOut =%d\r\n", u, nTxIn, nTxOut);
#endif
  if (ptx == NULL) blaster_alloc ();
  ptx->buf[ptx->len] = u;
  if (++ptx->len >= BLASTER_TX_SIZE) blaster_tx ();
}

void loop()
{
#if MEM_DEBUG > 0
  if (millis() >= tShow )
  {
    Serial2.printf ("time = %d\r\n", millis() / 1000);
    usb_mem_show();
    tShow = millis() + 10000;
  }
#endif
  if (usb_configuration == 0) return;
  
  usb_packet_t *prx = usb_rx (BLASTER_RX_EP);
  if ( prx != NULL )
  {
#if MEM_DEBUG > 0
    Serial2.printf ("prx = %p (%d)\r\n", prx, prx - pbase);
    usb_mem_show();
#endif
#if DEBUG > 0
    Serial2.printf ("Recv:");
    for (int i = 0; i < prx->len; ++i ) Serial2.printf (" %02X", prx->buf[i]);
    Serial2.printf ("\r\n");
#endif
    
    for (int i = 0; i < prx->len; ++i)
    {
      if ( nSeq > 0 )
      {
        uint8_t uSend = prx->buf[i];
        uint8_t uRecv = 0;
#if DEBUG > 1
        Serial2.printf ("JTAG Send: %02X, uPort = %02X, bRead = %d\r\n", uSend, uPort, bRead);
#endif
        for (int j = 0; j < 8; ++j)
        {
          if (bRead)
          {
            uRecv >>= 1;
            if (JTAG_RD () & uRead) uRecv |= 0x80;
          }
          if ( uSend & 0x01 ) uPort |= BIT_TDI;
          else uPort &= ~ BIT_TDI;
          uPort &= ~ BIT_TCK;
          JTAG_WR (uPort);
          uPort |= BIT_TCK;
          JTAG_WR (uPort);
          uPort &= ~ BIT_TCK;
          JTAG_WR (uPort);
          uSend >>= 1;
        }
        if ( bRead ) blaster_send (uRecv);
        --nSeq;
      }
      else
      {
        bRead = prx->buf[i] & BIT_RD;
        if ( prx->buf[i] & BIT_SEQ )
        {
          nSeq = prx->buf[i] & BITS_CNT;
          uPort &= ~ BIT_TCK;
          if ( uPort & BIT_NCS ) uRead = BIT_TDO;
          else uRead = BIT_ASO;
#if DEBUG > 1
          Serial2.printf ("prx->buf[%d] = %02X: nSeq = %d, uPort = %02X, bRead = %d\r\n",
            i, prx->buf[i], nSeq, uPort, bRead);
#endif
        }
        else
        {
#if DEBUG > 1
          Serial2.printf ("prx->buf[%d] = %02X:", i, prx->buf[i]);
          uint8_t uTmp = prx->buf[i];
          for (int i = 0; i < 8; ++i)
          {
            if ( uTmp & 0x01 ) Serial2.printf (" %s", psBits[i]);
            else Serial2.printf ("    ");
            uTmp >>= 1;
          }
          Serial2.printf ("\r\n");
#endif
#if SHOW_LED
          digitalWrite (PIN_LED, prx->buf[i] & BIT_ACT ? HIGH : LOW);
#endif
          uPort = prx->buf[i] & BITS_PORT;
          if ( bRead ) blaster_send (JTAG_RD ());
          JTAG_WR (uPort);
        }
      }
    }
    if (prx->len < 64)
    {
      blaster_tx ();
      blaster_alloc ();
      blaster_tx ();
      tNext = millis () + SEND_INT;
    }
#if MEM_DEBUG > 0
    usb_free (prx, 0);
#else
    usb_free (prx);
#endif
  }
  else if (millis () >= tNext)
  {
    blaster_alloc ();
    blaster_tx ();
    tNext = millis () + SEND_INT;
  }
}
