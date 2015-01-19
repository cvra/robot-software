/**
 * @file
 *
 * lwIP Options Configuration
 */

#ifndef __LWIPOPT_H__
#define __LWIPOPT_H__

/* See lwip/src/include/lwip/opt.h for reference. */

#define MEM_ALIGNMENT                   4

#define TCPIP_THREAD_STACKSIZE          1024
#define TCPIP_THREAD_PRIO               (LOWPRIO + 1)
#define TCPIP_MBOX_SIZE                 MEMP_NUM_PBUF

#define DEFAULT_THREAD_STACKSIZE        1024
#define DEFAULT_THREAD_PRIO             (LOWPRIO + 1)
#define DEFAULT_RAW_RECVMBOX_SIZE       4
#define DEFAULT_UDP_RECVMBOX_SIZE       4
#define DEFAULT_TCP_RECVMBOX_SIZE       40
#define DEFAULT_ACCEPTMBOX_SIZE         4

#define LWIP_IPADDR(p)  IP4_ADDR(p, 192, 168, 0, 20)
#define LWIP_GATEWAY(p) IP4_ADDR(p, 192, 168, 0, 1)
#define LWIP_NETMASK(p) IP4_ADDR(p, 255, 255, 255, 0)

#include "timestamp.h"

/* This macro is called everytime the NTP service wants to update the system time.
 * We use it to synchronize the unix time reference rather than changing
 * internal timers which are supposed to be only up counting. */
#define SNTP_SET_SYSTEM_TIME_US(sec, us) do { \
    unix_timestamp_t ts; \
    ts.s = sec; \
    ts.us = us; \
    timestamp_set_reference(ts, ST2US(chVTGetSystemTime())); \
    } while(0)

#endif /* __LWIPOPT_H__ */
