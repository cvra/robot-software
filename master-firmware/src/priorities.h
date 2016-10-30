#ifndef PRIORITIES_H_
#define PRIORITIES_H_

/* Higher number -> higher priority */

/* lwip threads */
#define TCPIP_THREAD_PRIO                       (LOWPRIO + 1)
#define LWIP_THREAD_PRIO                        (NORMALPRIO + 2)
#define DEFAULT_THREAD_PRIO                     (LOWPRIO + 1)

#define USB_SHELL_PRIO                          (NORMALPRIO + 2)
#define UAVCAN_PRIO                             (NORMALPRIO)
#define RPC_SERVER_PRIO                         (NORMALPRIO - 1)
#define ODOMETRY_PUBLISHER_PRIO                 (NORMALPRIO - 1)
#define DIFFERENTIAL_BASE_TRACKING_THREAD_PRIO  (NORMALPRIO - 1)
#define POSITION_MANAGER_PRIO                   (NORMALPRIO)
#define BASE_CONTROLLER_PRIO                    (NORMALPRIO)
#define TRAJECTORY_MANAGER_PRIO                 (NORMALPRIO)
#define STRATEGY_PRIO                           (NORMALPRIO + 2)
#define ENCODER_PRIO                            (NORMALPRIO)
#define STREAM_PRIO                             (NORMALPRIO + 1)
#define DYNAMIC_OBSTACLE_AVOIDANCE_PRIO         (NORMALPRIO + 2)

#define INTERFACE_PANEL_PRIO                    (NORMALPRIO + 2)

#endif
