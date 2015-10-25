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

#define TCPIP_THREAD_STACKSIZE          4096
#define TCPIP_MBOX_SIZE                 MEMP_NUM_PBUF

#define LWIP_DHCP 1

#define LWIP_SOCKET 0

#define DEFAULT_THREAD_STACK_SIZE       4096
#define DEFAULT_RAW_RECVMBOX_SIZE       4
#define DEFAULT_UDP_RECVMBOX_SIZE       4
#define DEFAULT_TCP_RECVMBOX_SIZE       4
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
    timestamp_set_reference(ts, timestamp_get()); \
    } while(0)

#define SNTP_SERVER_ADDRESS "192.168.3.1"

/** SNTP update delay - in milliseconds */
#define SNTP_UPDATE_DELAY (20 * 1000)

/** Use newlib malloc() instead of memory pools. */
#include <stdlib.h>
#define MEM_LIBC_MALLOC 1
#define MEMP_MEM_MALLOC 1



/* Robot settings. */
#ifdef ON_ROBOT

#define ODOMETRY_PUBLISHER_PORT 20000
#define ODOMETRY_PUBLISHER_HOST(server) ip_addr_set(server, IP_ADDR_BROADCAST)

#define STREAM_PORT            20042
#define STREAM_HOST(server)    ip_addr_set(server, IP_ADDR_BROADCAST)

#else
/* Laptop debug settings. We cannot use a broadcast address to debug over wifi
 * because routers don't forward broadcast packets. */

#define LAPTOP_IP(p) IP4_ADDR(p, 192, 168, 2, 101)


#define ODOMETRY_PUBLISHER_PORT 20000
#define ODOMETRY_PUBLISHER_HOST(server) LAPTOP_IP(server)

#define STREAM_PORT            20042
#define STREAM_HOST(server)    LAPTOP_IP(server)
//#define ENABLE_STREAM
#endif

#endif /* __LWIPOPT_H__ */
