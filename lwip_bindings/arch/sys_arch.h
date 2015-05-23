#ifndef SYS_ARCH_H
#define SYS_ARCH_H

#include <ch.h>
#include <lwip/sys.h>
#include <stdbool.h>

typedef struct {
    semaphore_t sem;
    bool is_valid;
} sys_sem_t;

#define SYS_SEM_NULL NULL

typedef struct {
} sys_mbox_t;

#define SYS_MBOX_NULL NULL

typedef struct {
} sys_mutex_t;

typedef struct {
} sys_thread_t;

/* Macros needed for atomicity. */
#define SYS_ARCH_DECL_PROTECT(x)
#define SYS_ARCH_PROTECT(x) chSysLock()
#define SYS_ARCH_UNPROTECT(x) chSysUnlock()

#endif
