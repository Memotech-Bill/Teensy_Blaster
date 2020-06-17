Altera "USB Blaster" emulation using Teensy 3.5
===============================================

The aim of this project was to use a Teensy 3.5 https://www.pjrc.com/store/teensy35.html
as a replacement for the Altera "USB Blaster" chip programmer.

For a project I am working on I wish to make use of an Altera EPM7032S chip.
These chips require a JTAG programmer to configure. I purchased a cheap
"USB Blaster" clone for the job. However for reasons yet to be determined,
although the Altera Quartus software recognised the clone as a "USB Blaster"
it failed to program the chip. This may have been caused by the programming
cable on the device having one of the connectors reversed when supplied,
resulting in damage to the JTAG drivers.

Not wishing to risk purchasing another dubious clone, I looked for alternatives.
There are existing designes for "USB Blaster" clones on the Net. However these
typically use a programable microcontroller such as a PIC18F14K50.
I don't have a programmer for these either.

My first attempt was to try and use a Raspberry Pi Zero in USB Gadget Mode
https://github.com/Memotech-Bill/Blaster_Gadget but I was unable to adequately
reproduce the required USB Interface. Therefore a Teensy 3.5 was tried instead.

Although my specific requirement was to be able to program an Altera EPM7032S chip,
the result could be used to program other Altera chips, or as a JTAG tap interface
for debugging using OpenOCD.

USB Blaster Emulation
=====================

USB Interface
-------------

In order to work the emulation would have to present the same USB interface
to the Quartus software as a real "USB Blaster". Connecting my failed "USB Blaster"
clone to a Linux machine, and running lsusb gave the following information as to the
interface:

        Device Descriptor:
         bLength                18
         bDescriptorType         1
         bcdUSB               1.10
         bDeviceClass            0 (Defined at Interface level)
         bDeviceSubClass         0 
         bDeviceProtocol         0 
         bMaxPacketSize0         8
         idVendor           0x09fb Altera
         idProduct          0x6001 Blaster
         bcdDevice            4.00
         iManufacturer           1 Altera
         iProduct                2 USB-Blaster
         iSerial                 3 00000000
         bNumConfigurations      1
         Configuration Descriptor:
           bLength                 9
           bDescriptorType         2
           wTotalLength           32
           bNumInterfaces          1
           bConfigurationValue     1
           iConfiguration          0 
           bmAttributes         0x80
             (Bus Powered)
           MaxPower               80mA
           Interface Descriptor:
             bLength                 9
             bDescriptorType         4
             bInterfaceNumber        0
             bAlternateSetting       0
             bNumEndpoints           2
             bInterfaceClass       255 Vendor Specific Class
             bInterfaceSubClass    255 Vendor Specific Subclass
             bInterfaceProtocol    255 Vendor Specific Protocol
             iInterface              0 
             Endpoint Descriptor:
               bLength                 7
               bDescriptorType         5
               bEndpointAddress     0x81  EP 1 IN
               bmAttributes            2
                 Transfer Type            Bulk
                 Synch Type               None
                 Usage Type               Data
               wMaxPacketSize     0x0040  1x 64 bytes
               bInterval               1
             Endpoint Descriptor:
               bLength                 7
               bDescriptorType         5
               bEndpointAddress     0x02  EP 2 OUT
               bmAttributes            2
                 Transfer Type            Bulk
                 Synch Type               None
                 Usage Type               Data
               wMaxPacketSize     0x0040  1x 64 bytes
               bInterval               1

Programming Protocol
--------------------

The following protocol definition was derived by reverse engineering the
software for the PIC18F14K50 based "USB Blaster" clone at
http://sa89a.net/mp.cgi/ele/ub.htm

Receive USB packets of up to 64 bytes from endpoint 2 containing command and data bytes as follows:

Command bytes:

1rnnnnnn    = Following nnnnnn data bytes are to be sent. If r is set, also read result.
              If /CS high, write & read JTAG, if /CS low write JTAG, read ASER.
              Also reset TCK to zero.

0r0xxxxx    = Bit bang the following pins:

* Bit RC0: TCK
* Bit RC1: TMS
* Bit RC2: /CE
* Bit RC3: /CS
* Bit RC4: TDI

If the read bit (bit 6) is set, then queue byte for return containing:

* Bit 0: TDO
* Bit 1: ASO

State read before changing output bits.

Data bytes:

JTAG bytes written to TDI low bit first. Clock taken high then low.
If required, results from TDO accumulated low bit first, before clock pulse.
Resulting bytes queued for return to host.

ASER bytes written to TDI low bit first. Clock taken high then low.
If required, results from ADO accumulated low bit first, before clock pulse.
Resulting bytes queued for return to host.

Sends 2-64 byte packets on endpoint 1 at least once every 10ms:

* Byte 0: 0x31
* Byte 1: 0x60
* Byte 2+: Data as above.

USB Setup Transactions
----------------------

The same source also identified the following USB setup transactions which have
to be emulated for the device to be recognised as a "USB Blaster":

Vendor Input Request 0x90 (144):

Request two bytes from the EEPROM of the FT245 chip in the original "USB Blaster".
The bIndex value in the request gives the address (divided by 2) of the bytes required.

Used by the Quartus software to confirm a "genuine USB Blaster".

Vendor Input - Other request values:

Return the two byte pair: 0x36, 0x83. The purpose of this is unknown.

Vendor Output - All request values:

Return a "Success" status, not "Stall".

Development
===========

Modify Teensy USB Core routines
-------------------------------

Following the documentation from Paul Stoffregen (reproduced as Teensy_USB.txt),
to implement the required USB interface it is necessary to edit and extend the
"usb_desc" and "usb_dev" files in the Teensy 3 core library. These modifications:

* Define a new USB type: "USB Blaster".
* Sepecifies all the USB descriptors for the "USB Blaster" interface.
* Process the Vendor specific setup requests as documented above.
* Call blaster_flush() for each USB frame.

Teensy_Blaster Sketch
---------------------

* Define statements at the top of the file specify which of the Teensy GPIO pins
are used to interface with the device to be programmed.
* Further defines help to implement the Blaster protocol.
* The setup() routine configures the GPIO pins then calls usb_init().
* Routines JTAG_WR() and JTAG_RD() implement the interface to the external hardware.
* Routine blaster_eeprom() returns bytes from the emulated FT245 EEPROM. These bytes
are defined in "eeprom.h", which was derived from the PIC chip software referenced above.
* Routine blaster_flush() submits any available output for transmission. If no available
data, increment flush counter.
* Routine blaster_alloc() allocates a new USB buffer for outgoing data and initialises
the first two bytes.
* Routine blaster_send() adds a byte of data to the buffer for transmission, and submits
the buffer if full.
* The main loop() routine:
  + Allocates a new transmission buffer if required.
  + Reads any available input data.
  + Implements the programming protocol.

Debugging
---------

It was quickly found that there was a small issue with the Teensy yield() function.
This assumed that there is always at least one USB serial port defined. Added #ifdef
conditions to resolve this.

Once the sketch was compiled and loaded into the Teensy, if it was run with extensive
diagnostics enabled it would sometimes work, and very slowly program the Altera chip.
However, if diagnostics were disabled, programming would hang.

After many days debugging the following was learned:

* The file usb_dev.c contains the comment "TODO: implement a per-endpoint maximum # of allocated
packets, so a flood of incoming data on 1 endpoint doesn't starve the others if the user isn't reading
it regularly". Quartus was sending so much data that the receiver was using all the usb_packet
buffers, leaving none to transmit results, resulting in a deadlock.

* I therefore decided implement multiple allocation pools of usb_packets. I chose to implement
one pool per endpoint. With hindsight it might have been better to implement one pool for transmit
and one for receive. For the "USB Blaster" application the effect is the same as the two endpoints
are unidirectional.

* usb_free() needed to be protected against bad packet addresses being returned.

* usb_free() was occasionally receiving valid packet addresses before they had been allocated!
I eventually realised that this was happening when I repeatedly loaded revised sketches, without
powering down the Teensy. It was remembering the allocation from the previous run. This was
due to a bug in the usb_init() routine in file "usb_dev.c". Line 1172 (in the original file) is:

        for (i=0; i <= NUM_ENDPOINTS*4; i++) {

It should be:

        for (i=0; i < (NUM_ENDPOINTS+1)*4; i++) {

As a result of this error the last three buffer descriptor tables were not being initialised.
Thus when a USB event occured (which resets all buffer descriptors), either a random address
or the address from the previous run was being returned to usb_free().

* It was not immediately obvious, but it was essential that usb_free() called usb_rx_memory().
If the receiver is not able to initially allocate memory, it does not retry. The result is a
deadlock, because no memory is allocated, there are no more read events, so no more memory
gets allocated without a call to usb_rx_memory(). Another option might be to retry memory
allocation during a start of frame event.

The resulting modified Teensy routines are in the "arduino" folder. Alternately
"teensy_blaster_arduino.patch" contains the patches that need to be applied to the
Teensyduino version 1.52 routines. The code is somewhat messy as I have left all my
diagnostic code in place.


Hardware
========

Since the Teensy 3.5 is a 3.3V device, with 5v tolerant inputs it can be connected directly
to either 3.3V or 5V chips for programming. Other voltage devices would require level shifters.

Given the absence of any significant additional hardware the programmer was assembled on a
breadboard. A 10K pulldown resistor was used on TCK, and 10K pullup resistors on TMS, TDI
and TDO.

A Teensy 3.5 was used for this project as I had it to hand. Given the small number of output
pins required, a Teensy 3.2 could probably be used instead, but this has not been tried.

References
==========

The following links have been of assistance in developing the code:

* http://sa89a.net/mp.cgi/ele/ub.htm
* https://translate.google.co.uk/translate?sl=auto&tl=en&u=http%3A%2F%2Fsa89a.net%2Fmp.cgi%2Fele%2Fub.htm
* https://forum.pjrc.com/threads/49045?p=164512&viewfull=1#post164512
* https://www.beyondlogic.org/usbnutshell/usb5.shtml
* https://www.beyondlogic.org/usbnutshell/usb6.shtml