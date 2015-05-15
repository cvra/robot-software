/**
 * @file
 *
 * lwIP Options Configuration
 */

#ifndef __LWIPOPT_H__
#define __LWIPOPT_H__

#include "priorities.h"

/* See lwip/src/include/lwip/opt.h for reference. */

#define MEM_ALIGNMENT                   4

#define TCPIP_THREAD_STACKSIZE          1024
#define TCPIP_MBOX_SIZE                 MEMP_NUM_PBUF

#define LWIP_DHCP 1

#define DEFAULT_THREAD_STACKSIZE        1024
#define DEFAULT_RAW_RECVMBOX_SIZE       4
#define DEFAULT_UDP_RECVMBOX_SIZE       4
#define DEFAULT_TCP_RECVMBOX_SIZE       40
#define DEFAULT_ACCEPTMBOX_SIZE         4

#define LWIP_IPADDR(p)  IP4_ADDR(p, 192, 168, 3, 20)
#define LWIP_GATEWAY(p) IP4_ADDR(p, 192, 168, 3, 1)
#define LWIP_NETMASK(p) IP4_ADDR(p, 255, 255, 255, 0)

#include "unix_timestamp.h"

/* This macro is called everytime the NTP service wants to update the system time.
 * We use it to synchronize the unix time reference rather than changing
 * internal timers which are supposed to be only up counting. */
#define SNTP_SET_SYSTEM_TIME_US(sec, us) do { \
    unix_timestamp_t ts; \
    ts.s = sec; \
    ts.us = us; \
    timestamp_set_reference(ts, ST2US(chVTGetSystemTime())); \
    } while(0)

/** Address of the SNTP server. Currently resovles to pool.ntp.org for
 * Switzerland. */
#define SNTP_SERVER_ADDRESS "192.168.2.1"

/** SNTP update delay - in milliseconds */
#define SNTP_UPDATE_DELAY (20 * 1000)

/** Use newlib malloc() instead of memory pools. */
#include <stdlib.h>
#define MEM_LIBC_MALLOC 1
#define MEMP_MEM_MALLOC 1


#endif /* __LWIPOPT_H__ */
