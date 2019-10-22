/**
 * @file
 *
 * lwIP Options Configuration
 */

#ifndef __LWIPOPT_H__
#define __LWIPOPT_H__

#include "priorities.h"

#define LWIP_DBG_TYPES_ON LWIP_DBG_ON
#define LWIP_COMPAT_MUTEX_ALLOWED

/* Disabling TCP reduces flash usage by aout 27 kB.*/
#define LWIP_TCP 0

/* See lwip/src/include/lwip/opt.h for reference. */

#define MEM_ALIGNMENT 4

#define TCPIP_THREAD_STACKSIZE 4096
#define TCPIP_MBOX_SIZE MEMP_NUM_PBUF

#define LWIP_SOCKET 0

#define DEFAULT_THREAD_STACK_SIZE 4096
#define DEFAULT_RAW_RECVMBOX_SIZE 4
#define DEFAULT_UDP_RECVMBOX_SIZE 4
#define DEFAULT_TCP_RECVMBOX_SIZE 4
#define DEFAULT_ACCEPTMBOX_SIZE 4

/* Deprecated, use parameter instead */
/* Set the default IP of each interface */
#define LWIP_ETHERNET_IPADDR(p) IP4_ADDR(p, 192, 168, 3, 20)
#define LWIP_ETHERNET_NETMASK(p) IP4_ADDR(p, 255, 255, 255, 0)
#define LWIP_CAN_IPADDR(p) IP4_ADDR(p, 192, 168, 4, 20)
#define LWIP_CAN_NETMASK(p) IP4_ADDR(p, 255, 255, 255, 0)
#define LWIP_GATEWAY(p) IP4_ADDR(p, 192, 168, 3, 1)

/** Use newlib malloc() instead of memory pools. */
#include <stdlib.h>
#define MEM_LIBC_MALLOC 1
#define MEMP_MEM_MALLOC 1

#define ODOMETRY_PUBLISHER_PORT 20042
#define STREAM_PORT 20042
#define BUTTON_PRESS_PUBLISHER_PORT 20042

#endif /* __LWIPOPT_H__ */
