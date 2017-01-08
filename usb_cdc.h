#ifndef _USB_CDC_
#define _USB_CDC_

extern unsigned char usb_putch(unsigned char ch);
extern void usb_putstr(char * s);
extern unsigned char usb_chReceived();
extern unsigned char usb_getch();
extern void usb_init();
extern void usb_process();
extern void usb_txprocess();
unsigned char usb_ep1_ready();
void usb_ep1_flush();
unsigned char usb_serialNumberAvailable();

const unsigned char usb_string_serial[] @ 0x0300;

#endif
