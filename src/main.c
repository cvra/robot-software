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
#include <arm-cortex-tools/mpu.h>
#include <arm-cortex-tools/fault.h>
#include "blocking_uart_driver.h"
#include "rpc_server.h"
#include "uavcan_node.h"
#include "timestamp/timestamp_stm32.h"
#include "config.h"
#include "interface_panel.h"
#include "robot_parameters.h"
#include "motor_manager.h"
#include "stream.h"
#include "malloc_lock.h"
#include <lwipthread.h>
#include "log.h"
#include "imu.h"
#include "usbconf.h"
#include "base/encoder.h"
#include "base/base_controller.h"

void init_base_motors(void);

/* Command line related */
#define SHELL_WA_SIZE   THD_WORKING_AREA_SIZE(2048)
static const ShellConfig shell_cfg1 = {
    (BaseSequentialStream *)&SDU1,
    commands
};

motor_manager_t motor_manager;

// debug UART
#define DEBUG_UART_BAUDRATE 921600
static const SerialConfig debug_uart_config = {
    .speed = DEBUG_UART_BAUDRATE,
    .cr1 = 0,
    .cr2 = USART_CR2_STOP1_BITS | USART_CR2_LINEN,
    .cr3 = 0
};

void fault_printf(const char *fmt, ...)
{
    va_list ap;
    va_start(ap, fmt);
    panic_log_vprintf(fmt, ap);
    va_end(ap);
}

/* Bus related declarations */
messagebus_t bus;
MUTEX_DECL(bus_lock);
CONDVAR_DECL(bus_condvar);

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
    if (ch.rlist.r_current != NULL) {
        panic_log_printf("\ncurrent thread: ");
        if (ch.rlist.r_current->p_name != NULL) {
            panic_log_printf("%s\n", ch.rlist.r_current->p_name);
        } else {
            panic_log_printf("0x%p\n", ch.rlist.r_current);
        }
    }
    BlockingUARTDriver panic_uart;
    blocking_uart_init(&panic_uart, USART3, DEBUG_UART_BAUDRATE);

    // block to preserve fault state
    const char *msg = panic_log_read();
    while(1) {
        if (msg != NULL) {
            chprintf((BaseSequentialStream *)&panic_uart, "kernel panic:\n%s\n", msg);
        }
        unsigned int i = 100000000;
        while(i--) {
            __asm__ volatile ("nop");
        }
    }
}

/** Late init hook, called before c++ static constructors. */
void __late_init(void)
{
    /* Enable fault handlers. */
    fault_init();

    /* Initalize memory protection unit and add a guard against NULL
     * dereferences. */
    mpu_init();

    /* C++ Static initializer requires working chibios. */
    halInit();
    chSysInit();
    malloc_lock_init();
}

/** Application entry point.  */
int main(void) {
    static thread_t *shelltp = NULL;

    /* Initializes a serial driver.  */
    sdStart(&SD3, &debug_uart_config);
    log_message("boot");

    /* Initializes a serial-over-USB CDC driver.  */
    sduObjectInit(&SDU1);
    sduStart(&SDU1, &serusbcfg);

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

    /* Initialize the interthread communication bus. */
    messagebus_init(&bus, &bus_lock, &bus_condvar);

    /* Initialize global objects. */
    config_init();


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


    struct netif *ethernet_if;

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
    imu_init();

    init_base_motors();

    /* Base init */
    encoder_start();
    robot_init();
    base_controller_start();
    position_manager_start();
    trajectory_manager_start();

    stream_init();

    /* main thread, spawns a shell on USB connection. */
    while (1) {
        if (!shelltp) {
            shelltp = shellCreate(&shell_cfg1, SHELL_WA_SIZE, USB_SHELL_PRIO);
        } else if (chThdTerminatedX(shelltp)) {
            chThdRelease(shelltp);    /* Recovers memory of the previous shell.   */
            shelltp = NULL;           /* Triggers spawning of a new shell.        */
        }

        chThdSleepMilliseconds(500);
    }
}

uintptr_t __stack_chk_guard = 0xdeadbeef;

void init_base_motors(void)
{
    motor_manager_create_driver(&motor_manager, "left-wheel");
    motor_manager_create_driver(&motor_manager, "right-wheel");

    /* Left wheel config */
    parameter_integer_set(PARAMETER("actuator/left-wheel/motor/mode"), 2);
    parameter_integer_set(PARAMETER("actuator/left-wheel/motor/motor_encoder_steps_per_revolution"), 4096);
    parameter_integer_set(PARAMETER("actuator/left-wheel/motor/second_encoder_steps_per_revolution"), 16384);
    parameter_integer_set(PARAMETER("actuator/left-wheel/motor/transmission_ratio_p"), 49);
    parameter_integer_set(PARAMETER("actuator/left-wheel/motor/transmission_ratio_q"), 676);
    parameter_scalar_set(PARAMETER("actuator/left-wheel/motor/torque_constant"), 1.);

    parameter_scalar_set(PARAMETER("actuator/left-wheel/control/current/kp"), 4.);
    parameter_scalar_set(PARAMETER("actuator/left-wheel/control/current/ki"), 1500.);
    parameter_scalar_set(PARAMETER("actuator/left-wheel/control/current/kd"), 0.);
    parameter_scalar_set(PARAMETER("actuator/left-wheel/control/current/ilimit"), 50.);

    parameter_scalar_set(PARAMETER("actuator/left-wheel/control/velocity/kp"), 14.);
    parameter_scalar_set(PARAMETER("actuator/left-wheel/control/velocity/ki"), 110.);
    parameter_scalar_set(PARAMETER("actuator/left-wheel/control/velocity/kd"), 0.);
    parameter_scalar_set(PARAMETER("actuator/left-wheel/control/velocity/ilimit"), 1400.);

    parameter_scalar_set(PARAMETER("actuator/left-wheel/control/position/kp"), 0.1);
    parameter_scalar_set(PARAMETER("actuator/left-wheel/control/position/ki"), 0.5);
    parameter_scalar_set(PARAMETER("actuator/left-wheel/control/position/kd"), 0.);

    parameter_scalar_set(PARAMETER("actuator/left-wheel/control/low_batt_th"), 5.);
    parameter_scalar_set(PARAMETER("actuator/left-wheel/control/acceleration_limit"), 100.);
    parameter_scalar_set(PARAMETER("actuator/left-wheel/control/velocity_limit"), 50.);
    parameter_scalar_set(PARAMETER("actuator/left-wheel/control/torque_limit"), 14.);

    /* Right wheel config */
    parameter_integer_set(PARAMETER("actuator/right-wheel/motor/mode"), 2);
    parameter_integer_set(PARAMETER("actuator/right-wheel/motor/motor_encoder_steps_per_revolution"), 4096);
    parameter_integer_set(PARAMETER("actuator/right-wheel/motor/second_encoder_steps_per_revolution"), 16384);
    parameter_integer_set(PARAMETER("actuator/right-wheel/motor/transmission_ratio_p"), 49);
    parameter_integer_set(PARAMETER("actuator/right-wheel/motor/transmission_ratio_q"), 676);
    parameter_scalar_set(PARAMETER("actuator/right-wheel/motor/torque_constant"), 1.);

    parameter_scalar_set(PARAMETER("actuator/right-wheel/control/current/kp"), 4.);
    parameter_scalar_set(PARAMETER("actuator/right-wheel/control/current/ki"), 1500.);
    parameter_scalar_set(PARAMETER("actuator/right-wheel/control/current/kd"), 0.);
    parameter_scalar_set(PARAMETER("actuator/right-wheel/control/current/ilimit"), 50.);

    parameter_scalar_set(PARAMETER("actuator/right-wheel/control/velocity/kp"), 14.);
    parameter_scalar_set(PARAMETER("actuator/right-wheel/control/velocity/ki"), 110.);
    parameter_scalar_set(PARAMETER("actuator/right-wheel/control/velocity/kd"), 0.);
    parameter_scalar_set(PARAMETER("actuator/right-wheel/control/velocity/ilimit"), 1400.);

    parameter_scalar_set(PARAMETER("actuator/right-wheel/control/position/kp"), 0.1);
    parameter_scalar_set(PARAMETER("actuator/right-wheel/control/position/ki"), 0.5);
    parameter_scalar_set(PARAMETER("actuator/right-wheel/control/position/kd"), 0.);

    parameter_scalar_set(PARAMETER("actuator/right-wheel/control/low_batt_th"), 5.);
    parameter_scalar_set(PARAMETER("actuator/right-wheel/control/acceleration_limit"), 100.);
    parameter_scalar_set(PARAMETER("actuator/right-wheel/control/velocity_limit"), 50.);
    parameter_scalar_set(PARAMETER("actuator/right-wheel/control/torque_limit"), 14.);
}

void __stack_chk_fail(void)
{
    chSysHalt("Stack smashing detected");
}

void context_switch_hook(void *ntp, void *otp)
{
    (void) otp;

    /* The main thread does not have the same memory layout as the other ones
       (it uses the process stack instead of its own stack), so we ignore it. */
    if (ntp == &ch.mainthread) {
        return;
    }

    /* Note: We want to use mpu_configure_region inside a thread
       or an ISR context. It turns out ChibiOS doesn't like it (panic)
       if you lock around mpu_configure_region in here. */
    mpu_configure_region(6,
                         /* we skip sizeof(thread_t) because the start of the working area is used by ChibiOS. */
                         ntp + sizeof(thread_t) + 32,
                         5, /* 32 bytes */
                         AP_NO_NO, /* no permission */
                         false);
}
