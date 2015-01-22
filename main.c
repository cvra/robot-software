#include <stdio.h>
#include <string.h>

#include <hal.h>
#include <chprintf.h>
#include <lwipthread.h>
#include <lwip/netif.h>

#include "commands.h"
#include "sntp/sntp.h"
#include "timestamp.h"
#include "panic_log.h"
#include "usb_shell.h"

#define USB_SHELL_STACK_SIZE 512
/** Stack for the USB shell task. */
THD_WORKING_AREA(stack_usb_shell, USB_SHELL_STACK_SIZE);

/**
 * Function called on a kernel panic.
 * @param [in] reaon Kernel panic message.  */
void panic_hook(const char *reason)
{
    panic_log_write(reason);

    // reboot
    NVIC_SystemReset();
}

/** Application entry point.  */
int main(void) {
    /*
     * System initializations.
     * - HAL initialization, this also initializes the configured device drivers
     *   and performs the board-specific initializations.
     * - Kernel initialization, the main() function becomes a thread and the
     *   RTOS is active.
     */
    halInit();
    chSysInit();

    /* Shell manager initialization.  */
    shellInit();

    /* Checks if there is any log message from a previous boot */
    if (panic_log_read() != NULL) {
        /* Turns on the user LED if yes */
        palClearPad(GPIOC, GPIOC_LED);
    }

    /* Creates the LWIP threads (it changes priority internally).  */
    chThdCreateStatic(wa_lwip_thread, LWIP_THREAD_STACK_SIZE, NORMALPRIO + 2,
            lwip_thread, NULL);

    /* Starts clock network sync. */
    sntp_init();

    /* Creates the USB shell thread */
    chThdCreateStatic(stack_usb_shell, USB_SHELL_STACK_SIZE, NORMALPRIO + 2,
            usb_shell_thread, NULL);

    while (1) {
    }
}
