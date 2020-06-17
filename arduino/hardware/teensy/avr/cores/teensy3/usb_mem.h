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

#ifndef _usb_mem_h_
#define _usb_mem_h_

//  Debug levels:
//      0 = No debugging
//      1 = Collect information
//      2 = Messages during interrupt routines
#define MEM_DEBUG   0

#include <stdint.h>
#include "usb_desc.h"

// It seems that buf must be 32 bit aligned. Therefore this structure
// must be a multiple of 32 bits long.
typedef struct usb_packet_struct {
	uint16_t len;
	uint16_t index;
	struct usb_packet_struct *next;
	uint8_t buf[64];
#ifdef USB_POOL
    int iPool;
#endif
} usb_packet_t;

#ifdef __cplusplus
extern "C" {
#endif

#ifdef USB_POOL
int usb_mem_init(void);
#if MEM_DEBUG > 0
usb_packet_t * usb_malloc(int iEP, int iLine);
void usb_free(usb_packet_t *p, int iLine);
#else
usb_packet_t * usb_malloc(int iEP);
void usb_free(usb_packet_t *p);
#endif
#else // USB_POOL not defined
usb_packet_t * usb_malloc(void);
void usb_free(usb_packet_t *p);
#endif

void UsbLog (const char *psFmt,...);
#if MEM_DEBUG > 0
const usb_packet_t * usb_mem_base(void);
void usb_mem_show(void);
void usb_queues(void);
#endif

#ifdef __cplusplus
}
#endif


#endif
