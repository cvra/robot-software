#ifndef USB_SHELL_C_
#define USB_SHELL_C_

#include "hal.h"

/** Thread function keeping a shell open on the USB to serial port. */
msg_t usb_shell_thread(void *dummy);

#endif
