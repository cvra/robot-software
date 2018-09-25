#include <stdio.h>
#include <string.h>
#include <math.h>

#include <hal.h>
#include <chprintf.h>

#include <arm-cortex-tools/mpu.h>
#include <arm-cortex-tools/fault.h>
#include <cmp_mem_access/cmp_mem_access.h>
#include <error/error.h>
#include <parameter/parameter_msgpack.h>
#include <shell.h>
#include <timestamp/timestamp_stm32.h>

#include "main.h"
#include "unix_timestamp.h"
#include "panic_log.h"
#include "log.h"
#include "blocking_uart.h"
#include "uavcan_node.h"
#include "config.h"
#include "motor_manager.h"
#include "malloc_lock.h"
#include "usbconf.h"
#include "pca9685_pwm.h"
#include "commands.h"
#include "remote_control.h"

void init_base_motors(void);

motor_manager_t motor_manager;
base_t rover_base;

// debug UART
#define DEBUG_UART_BAUDRATE 115200
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

/**
 * Function called on a kernel panic.
 * @param [in] reaon Kernel panic message.  */
void panic_hook(const char *reason)
{
    palSetPad(GPIOB, GPIOB_LED_GREEN);
    palSetPad(GPIOB, GPIOB_LED_RED);

    panic_log_write(reason);
    if (ch.rlist.current != NULL) {
        panic_log_printf("\ncurrent thread: ");
        if (ch.rlist.current->name != NULL) {
            panic_log_printf("%s\n", ch.rlist.current->name);
        } else {
            panic_log_printf("0x%p\n", ch.rlist.current);
        }
    }
    BlockingUARTDriver panic_uart;
    blocking_uart_init(&panic_uart, UART7, DEBUG_UART_BAUDRATE);

    // block to preserve fault state
    const char *msg = panic_log_read();
    while (1) {
        if (msg != NULL) {
            chprintf((BaseSequentialStream *)&panic_uart, "kernel panic:\n%s\n", msg);
        }
        unsigned int i = 100000000;
        while (i--) {
            __asm__ volatile ("nop");
        }
    }
}

void _unhandled_exception(void)
{
    chSysHalt("unhandled exception");

    while (true) {
        /* wait forever */
    }
}

void _fini(void)
{
    /* empty */
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

void config_load_err_cb(void *arg, const char *id, const char *err)
{
    (void)arg;
    WARNING("parameter %s: %s", id == NULL ? "(...)" : id, err);
}

extern unsigned char msgpack_config_rover[];
extern const size_t msgpack_config_rover_size;

void config_load_from_flash(void)
{
    cmp_ctx_t cmp;
    cmp_mem_access_t mem;
    cmp_mem_access_ro_init(&cmp, &mem, msgpack_config_rover, msgpack_config_rover_size);
}

/** Application entry point.  */
int main(void)
{
    /* Initializes a serial driver.  */
    sdStart(&SD7, &debug_uart_config);

    /* Initialize global objects. */
    config_init();

    log_init();

    NOTICE("boot");

    /* Initialise timestamp module */
    timestamp_stm32_init();

    pca9685_pwm_init(0.0212);

    /* bus enumerator init */
    static __attribute__((section(".ccm"))) struct bus_enumerator_entry_allocator
        bus_enum_entries_alloc[MAX_NB_BUS_ENUMERATOR_ENTRIES];

    bus_enumerator_init(&bus_enumerator,
                        bus_enum_entries_alloc,
                        MAX_NB_BUS_ENUMERATOR_ENTRIES);


    static __attribute__((section(".ccm"))) motor_driver_t motor_driver_buffer[MAX_NB_MOTOR_DRIVERS];

    motor_manager_init(&motor_manager,
                       motor_driver_buffer,
                       MAX_NB_MOTOR_DRIVERS,
                       &bus_enumerator);

    /* Initialize motors */
    init_base_motors();

    /* Load stored robot config */
    config_load_from_flash();

    /* Initiaze UAVCAN communication */
    uavcan_node_start(10);

    remote_control_start();

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

    /* main thread, spawns a shell on USB connection. */
    while (1) {
        shell_spawn((BaseSequentialStream *)&SDU1);
        chThdSleepMilliseconds(500);
    }
}

uintptr_t __stack_chk_guard = 0xdeadbeef;

void init_base_motors(void)
{
    rover_base.left.back_wheel.motor = motor_manager_create_driver(&motor_manager, "left-back-wheel");
    rover_base.left.center_wheel.motor = motor_manager_create_driver(&motor_manager, "left-center-wheel");
    rover_base.left.front_wheel.motor = motor_manager_create_driver(&motor_manager, "left-front-wheel");
    rover_base.left.back_wheel.speed_factor = rover_base.left.center_wheel.speed_factor = rover_base.left.front_wheel.speed_factor = 1.f;

    rover_base.left.back_wheel.servo = 0;
    rover_base.left.center_wheel.servo = 2;
    rover_base.left.front_wheel.servo = 4;
    rover_base.left.back_wheel.steering_center = 0.0015f;
    rover_base.left.center_wheel.steering_center = 0.0015f;
    rover_base.left.front_wheel.steering_center = 0.0015f;

    rover_base.right.back_wheel.motor = motor_manager_create_driver(&motor_manager, "right-back-wheel");
    rover_base.right.center_wheel.motor = motor_manager_create_driver(&motor_manager, "right-center-wheel");
    rover_base.right.front_wheel.motor = motor_manager_create_driver(&motor_manager, "right-front-wheel");
    rover_base.right.back_wheel.speed_factor = rover_base.right.center_wheel.speed_factor = rover_base.right.front_wheel.speed_factor = -1.f;

    rover_base.right.back_wheel.servo = 6;
    rover_base.right.center_wheel.servo = 8;
    rover_base.right.front_wheel.servo = 10;
    rover_base.right.back_wheel.steering_center = 0.0015f;
    rover_base.right.center_wheel.steering_center = 0.0015f;
    rover_base.right.front_wheel.steering_center = 0.0015f;
}

void __stack_chk_fail(void)
{
    ERROR("Stack smashing detected");
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

    const char *name = ((thread_t *)ntp)->name;
    if (name == NULL) {
        name = "no name";
    }
}
