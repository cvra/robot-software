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

#endif /* __LWIPOPT_H__ */
