#include <stdio.h>
#include <string.h>
#include <math.h>

#include <hal.h>
#include <chprintf.h>
#include <lwipthread.h>
#include <lwip/netif.h>
#include <lwip/dhcp.h>

#include "main.h"
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
#include "motor_control.h"
#include "robot_pose.h"
#include "tracy-the-trajectory-tracker/src/trajectory_tracking.h"
#include "robot_parameters.h"
#include "odometry_publisher.h"
#include "motor_manager.h"


/* Command line related.                                                     */
#define SHELL_WA_SIZE   THD_WORKING_AREA_SIZE(2048)

#define TRAJECTORY_STACKSIZE 2048


motor_manager_t motor_manager;


THD_WORKING_AREA(wa_trajectory, TRAJECTORY_STACKSIZE);

static const ShellConfig shell_cfg1 = {
    (BaseSequentialStream *)&SDU1,
    commands
};

/**
 * Function called on a kernel panic.
 * @param [in] reaon Kernel panic message.  */
void panic_hook(const char *reason)
{
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

//    panic_log_write(reason);

    // reboot
    //NVIC_SystemReset();
    while(1);
}

msg_t trajectory_thread(void *p)
{
    (void) p;

    static float trajectory_buffer[100][5];
    trajectory_init(&robot_trajectory, (float *)trajectory_buffer, 100, 5, 100*1000);

    chMtxObjectInit(&robot_trajectory_lock);

    while (1) {
        float *point;
        float x, y, theta, speed, omega;
        uint64_t now;

        now = ST2US(chVTGetSystemTime());

        chMtxLock(&robot_trajectory_lock);
        point = trajectory_read(&robot_trajectory, now);
        chMtxUnlock(&robot_trajectory_lock);

        if (point) {
            struct tracking_error error;
            struct robot_velocity input, output;

            x = point[0];
            y = point[1];
            theta = point[2];
            speed = point[3];
            omega = point[4];

            /* Get data from odometry. */
            chMtxLock(&robot_pose_lock);
                error.x_error = x - robot_pose.x;
                error.y_error = y - robot_pose.y;
                error.theta_error = theta - robot_pose.theta;

                theta = robot_pose.theta;
            chMtxUnlock(&robot_pose_lock);

            input.tangential_velocity = speed;
            input.angular_velocity = omega;

            /* Transform error to local frame. */
            tracy_global_error_to_local(&error, theta);

            /* Perform controller iteration */
            tracy_linear_controller(&error, &input, &output);

            /* Apply speed to wheels. */
            chprintf((BaseSequentialStream *)&SDU1 , "%d %.2f %.2f\n\r", now, output.tangential_velocity, output.angular_velocity);


            /* TODO: Refactor this */
            m1_vel_setpt = (0.5f * ROBOT_RIGHT_WHEEL_DIRECTION / ROBOT_RIGHT_MOTOR_WHEEL_RADIUS) * (output.tangential_velocity / M_PI + ROBOT_MOTOR_WHEELBASE * output.angular_velocity);
            m2_vel_setpt = (0.5f * ROBOT_LEFT_WHEEL_DIRECTION / ROBOT_LEFT_MOTOR_WHEEL_RADIUS) * (output.tangential_velocity / M_PI - ROBOT_MOTOR_WHEELBASE * output.angular_velocity);

        }

        chThdSleepMilliseconds(100);
    }

    return MSG_OK;
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
        /* Creates the LWIP threads (it changes priority internally).  */
        chThdCreateStatic(wa_lwip_thread, LWIP_THREAD_STACK_SIZE, NORMALPRIO + 2,
            lwip_thread, NULL);

        chThdCreateStatic(wa_trajectory,
                      TRAJECTORY_STACKSIZE,
                      RPC_SERVER_PRIO,
                      trajectory_thread,
                      NULL);


        chThdSleepMilliseconds(1000);
        ethernet_if = netif_find("ms0");
        if (ethernet_if) {
            dhcp_start(ethernet_if);
        }

        sntp_init();
        can_bridge_init();
        uavcan_node_start(10);
        rpc_server_init();
        message_server_init();
        interface_panel_init();
        odometry_publisher_init();
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
