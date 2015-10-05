#include <stdio.h>
#include <string.h>
#include <math.h>

#include <hal.h>
#include <chprintf.h>
#include <lwip/netif.h>
#include <lwip/dhcp.h>

#include "main.h"
#include "commands.h"
#include "sntp/sntp.h"
#include "unix_timestamp.h"
#include "panic_log.h"
#include "rpc_server.h"
#include "uavcan_node.h"
#include "timestamp/timestamp_stm32.h"
#include "usbconf.h"
#include "config.h"
#include "interface_panel.h"
#include "robot_pose.h"
#include "robot_parameters.h"
#include "odometry_publisher.h"
#include "motor_manager.h"
#include "differential_base.h"
#include "stream.h"
#include "malloc_lock.h"
#include <lwipthread.h>

#if defined(DEBUG)
#warning DEBUG is defined, remove for match!
#endif

/* Command line related.                                                     */
#define SHELL_WA_SIZE   THD_WORKING_AREA_SIZE(2048)
static const ShellConfig shell_cfg1 = {
    (BaseSequentialStream *)&SDU1,
    commands
};

motor_manager_t motor_manager;

// debug UART
#define DEBUG_UART_BAUDRATE 119200
static const SerialConfig debug_uart_config = {
    .speed = DEBUG_UART_BAUDRATE,
    .cr1 = 0,
    .cr2 = USART_CR2_STOP1_BITS | USART_CR2_LINEN,
    .cr3 = 0
};

/**
 * Function called on a kernel panic.
 * @param [in] reaon Kernel panic message.  */
void panic_hook(const char *reason)
{
    (void) reason;
    palClearPad(GPIOC, GPIOC_LED);
    palSetPad(GPIOF, GPIOF_LED_READY);
    palSetPad(GPIOF, GPIOF_LED_DEBUG);
    palSetPad(GPIOF, GPIOF_LED_ERROR);
    palSetPad(GPIOF, GPIOF_LED_POWER_ERROR);
    palSetPad(GPIOF, GPIOF_LED_PC_ERROR);
    palSetPad(GPIOF, GPIOF_LED_BUS_ERROR);
    palSetPad(GPIOF, GPIOF_LED_YELLOW_1);
    palSetPad(GPIOF, GPIOF_LED_YELLOW_2);
    palSetPad(GPIOF, GPIOF_LED_GREEN_1);
    palSetPad(GPIOF, GPIOF_LED_GREEN_2);

    panic_log_write(reason);
#if defined(DEBUG)
    BlockingUARTDriver panic_uart;
    blocking_uart_init(&panic_uart, USART3, DEBUG_UART_BAUDRATE);

    // block to preserve fault state
    const char *msg = panic_log_read();
    while(1) {
        if (msg != NULL) {
            chprintf((BaseSequentialStream *)&panic_uart, "kernel panic:\n%s\n", msg);
        }
        unsigned int i = 10000000;
        while(i--) {
            __asm__ volatile ("nop");
        }
    }
#else
    // reboot
    NVIC_SystemReset();
#endif
}

/** Late init hook, called before c++ static constructors. */
void __late_init(void)
{
    /* C++ Static initializer requires working chibios. */
    halInit();
    chSysInit();
    malloc_lock_init();
}

/** Application entry point.  */
int main(void) {
    static thread_t *shelltp = NULL;


    /* Initializes a serial-over-USB CDC driver.  */
    sduObjectInit(&SDU1);
    sduStart(&SDU1, &serusbcfg);

    sdStart(&SD3, &debug_uart_config);
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
    chMtxObjectInit(&robot_pose_lock);


    /* Initialise timestamp module */
    timestamp_stm32_init();


    /* bus enumerator init */
    static __attribute__((section(".ccm"))) struct bus_enumerator_entry_allocator
                    bus_enum_entries_alloc[MAX_NB_BUS_ENUMERATOR_ENTRIES];

    bus_enumerator_init(&bus_enumerator,
                        bus_enum_entries_alloc,
                        MAX_NB_BUS_ENUMERATOR_ENTRIES);


    /* allocate and init motor manager */
    static __attribute__((section(".ccm"))) trajectory_t trajectory_buffer[MAX_NB_TRAJECTORY_BUFFERS];
    static __attribute__((section(".ccm"))) float trajectory_points_buffer[ACTUATOR_TRAJECTORY_NB_POINTS
                                                                           * ACTUATOR_TRAJECTORY_POINT_DIMENSION
                                                                           * MAX_NB_TRAJECTORY_BUFFERS];

    static __attribute__((section(".ccm"))) motor_driver_t motor_driver_buffer[MAX_NB_MOTOR_DRIVERS];

    motor_manager_init(&motor_manager,
                       trajectory_buffer,
                       MAX_NB_TRAJECTORY_BUFFERS,
                       trajectory_points_buffer,
                       MAX_NB_TRAJECTORY_BUFFERS,
                       motor_driver_buffer,
                       MAX_NB_MOTOR_DRIVERS,
                       &bus_enumerator);

    differential_base_init();

    /* Checks if there is any log message from a previous boot */
    if (panic_log_read() != NULL) {
        /* Turns on the user LED if yes */
        palClearPad(GPIOC, GPIOC_LED);

        /* Turn on all LEDs on the front panel. */
        palSetPad(GPIOF, GPIOF_LED_READY);
        palSetPad(GPIOF, GPIOF_LED_DEBUG);
        palSetPad(GPIOF, GPIOF_LED_ERROR);
        palSetPad(GPIOF, GPIOF_LED_POWER_ERROR);
        palSetPad(GPIOF, GPIOF_LED_PC_ERROR);
        palSetPad(GPIOF, GPIOF_LED_BUS_ERROR);
        palSetPad(GPIOF, GPIOF_LED_YELLOW_1);
        palSetPad(GPIOF, GPIOF_LED_YELLOW_2);
        palSetPad(GPIOF, GPIOF_LED_GREEN_1);
        palSetPad(GPIOF, GPIOF_LED_GREEN_2);
    } else {
        struct netif *ethernet_if;

        differential_base_tracking_start(); // tracy
        ip_thread_init();

        chThdSleepMilliseconds(1000);
        ethernet_if = netif_find("ms0");
        if (ethernet_if) {
            dhcp_start(ethernet_if);
        }

        sntp_init();
        uavcan_node_start(10);
        rpc_server_init();
        message_server_init();
        interface_panel_init();
        odometry_publisher_init();

#ifdef ENABLE_STREAM
        #warning "Enabling robot stream can lead to lwip crash. Do not use in match until fixed."
        stream_init();
#endif
    }

    /* main thread, spawns a shell on USB connection. */
    while (1) {
        if (!shelltp && (SDU1.config->usbp->state == USB_ACTIVE)) {
            shelltp = shellCreate(&shell_cfg1, SHELL_WA_SIZE, USB_SHELL_PRIO);
        } else if (chThdTerminatedX(shelltp)) {
            chThdRelease(shelltp);    /* Recovers memory of the previous shell.   */
            shelltp = NULL;           /* Triggers spawning of a new shell.        */
        }

        chThdSleepMilliseconds(500);
    }
}

