#ifndef SYS_ARCH_H
#define SYS_ARCH_H

#include <ch.h>
#include <lwip/sys.h>
#include <stdbool.h>

/* Number of items stored in a message queue. */
#define SYS_ARCH_Q_SIZE TCPIP_MBOX_SIZE

typedef struct {
    semaphore_t sem;
    bool is_valid;
} sys_sem_t;

#define SYS_SEM_NULL NULL

typedef struct {
    mailbox_t mb;
    msg_t buf[SYS_ARCH_Q_SIZE];
    bool is_valid;
} sys_mbox_t;

#define SYS_MBOX_NULL NULL

/* Use semaphores as mutexes */
#define LWIP_COMPAT_MUTEX 1

typedef thread_t *sys_thread_t;

/* Macros needed for atomicity. */
#define SYS_ARCH_DECL_PROTECT(x)
#define SYS_ARCH_PROTECT(x) chSysLock()
#define SYS_ARCH_UNPROTECT(x) chSysUnlock()

#endif
