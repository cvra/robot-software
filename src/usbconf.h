#ifndef USBCONF_H
#define USBCONF_H

#include <hal.h>

extern SerialUSBDriver SDU1;

/* Starts the UART over USB service. */
void usb_start(void);

#endif
