#ifndef PRIORITIES_H_
#define PRIORITIES_H_

/* Higher number -> higher priority */

/* lwip threads */
#define TCPIP_THREAD_PRIO                       (LOWPRIO + 1)
#define DEFAULT_THREAD_PRIO                     (LOWPRIO + 1)

#define USB_SHELL_PRIO                          (NORMALPRIO + 2)
#define CAN_BRIDGE_PRIO                         (NORMALPRIO - 1)
#define RPC_SERVER_PRIO                         (NORMALPRIO - 1)
#define ODOMETRY_PUBLISHER_PRIO                 (NORMALPRIO - 1)
#define DIFFERENTIAL_BASE_TRACKING_THREAD_PRIO  (NORMALPRIO - 1)
#define STREAM_PRIO                             (NORMALPRIO - 3)

#define INTERFACE_PANEL_PRIO                    (NORMALPRIO + 2)

#endif
