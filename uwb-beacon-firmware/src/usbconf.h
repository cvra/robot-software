#ifndef USBCONF_H
#define USBCONF_H

#include <hal.h>

extern SerialUSBDriver SDU1;

/** Starts the UART over USB service.
 *
 * @parameter serial [in] serial number to use for the port. */
void usb_start(unsigned int serial);

#endif
