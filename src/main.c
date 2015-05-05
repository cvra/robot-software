#include <stdio.h>
#include <string.h>

#include <hal.h>
#include <chprintf.h>
#include <lwipthread.h>
#include <lwip/netif.h>

#include "commands.h"
#include "sntp/sntp.h"
#include "unix_timestamp.h"
#include "panic_log.h"
#include "can_bridge.h"
#include "rpc_server.h"
#include "uavcan_node.h"
#include "timestamp/timestamp_stm32.h"
#include "usbconf.h"
#include "config.h"
#include "interface_panel.h"

/* Command line related.                                                     */
#define SHELL_WA_SIZE   THD_WORKING_AREA_SIZE(2048)

static const ShellConfig shell_cfg1 = {
    (BaseSequentialStream *)&SDU1,
    commands
};

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
    static thread_t *shelltp = NULL;

    /*
     * System initializations.
     * - HAL initialization, this also initializes the configured device drivers
     *   and performs the board-specific initializations.
     * - Kernel initialization, the main() function becomes a thread and the
     *   RTOS is active.
     */
    halInit();
    chSysInit();

    /* Initializes a serial-over-USB CDC driver.  */
    sduObjectInit(&SDU1);
    sduStart(&SDU1, &serusbcfg);

    sdStart(&SD3, NULL);
    chprintf((BaseSequentialStream *)&SD3 , "\n> boot\n");

    /*
     * Activates the USB driver and then the USB bus pull-up on D+.
     * Note, a delay is inserted in order to not have to disconnect the cable
     * after a reset.
     */
    usbDisconnectBus(serusbcfg.usbp);
    chThdSleepMilliseconds(1500);
    usbStart(serusbcfg.usbp, &usbcfg);
    usbConnectBus(serusbcfg.usbp);

    /* Shell manager initialization.  */
    shellInit();

    /* Initialize global objects. */
    config_init();


    /* Initialise timestamp module */
    timestamp_stm32_init();

    /* Checks if there is any log message from a previous boot */
    if (panic_log_read() != NULL) {
        /* Turns on the user LED if yes */
        palClearPad(GPIOC, GPIOC_LED);
    }

    if (panic_log_read() == NULL) {
        /* Creates the LWIP threads (it changes priority internally).  */
        chThdCreateStatic(wa_lwip_thread, LWIP_THREAD_STACK_SIZE, NORMALPRIO + 2,
            lwip_thread, NULL);

        sntp_init();
        can_bridge_init();
        uavcan_node_start(42);
        rpc_server_init();
        message_server_init();
        interface_panel_init();
    }

    /* main thread, spawns a shell on USB connection. */
    while (1) {
        if (!shelltp && (SDU1.config->usbp->state == USB_ACTIVE)) {
            shelltp = shellCreate(&shell_cfg1, SHELL_WA_SIZE, NORMALPRIO);
        } else if (chThdTerminatedX(shelltp)) {
            chThdRelease(shelltp);    /* Recovers memory of the previous shell.   */
            shelltp = NULL;           /* Triggers spawning of a new shell.        */
        }

        chThdSleepMilliseconds(500);
    }
}
