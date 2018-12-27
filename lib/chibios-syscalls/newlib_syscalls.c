#include <stdlib.h>
#include <errno.h>
#include <string.h>
#include <sys/stat.h>
#include <sys/types.h>

#include "ch.h"

#if defined(STDOUT_SD) || defined(STDIN_SD)
#include "hal.h"
extern BaseSequentialStream STDOUT_SD;
extern BaseSequentialStream STDIN_SD;
#endif

int _read_r(struct _reent *r, int file, char * ptr, int len)
{
    (void)r;
#if defined(STDIN_SD)
    if (!len || (file != 0)) {
        __errno_r(r) = EINVAL;
        return -1;
    }
    len = chSequentialStreamRead((BaseSequentialStream *)&STDOUT_SD, (uint8_t *)ptr, len);
    return len;
#else
    (void)file;
    (void)ptr;
    (void)len;
    __errno_r(r) = EINVAL;
    return -1;
#endif
}

int _lseek_r(struct _reent *r, int file, int ptr, int dir)
{
    (void)r;
    (void)file;
    (void)ptr;
    (void)dir;

    return 0;
}

int _write_r(struct _reent *r, int file, char * ptr, int len)
{
    (void)r;
    (void)file;
    (void)ptr;
#if defined(STDOUT_SD)
    if (file != 1) {
        __errno_r(r) = EINVAL;
        return -1;
    }
    chSequentialStreamWrite((BaseSequentialStream *)&STDOUT_SD, (uint8_t *)ptr, (size_t)len);
#endif
    return len;
}

int _close_r(struct _reent *r, int file)
{
    (void)r;
    (void)file;

    return 0;
}

caddr_t _sbrk_r(struct _reent *r, int incr)
{
#if CH_CFG_USE_MEMCORE
    void *p;

    chDbgCheck(incr > 0);

    p = chCoreAlloc((size_t)incr);
    if (p == NULL) {
        __errno_r(r) = ENOMEM;
        return (caddr_t)-1;
    }
    return (caddr_t)p;
#else
    (void)incr;
    __errno_r(r) = ENOMEM;
    return (caddr_t)-1;
#endif
}

int _fstat_r(struct _reent *r, int file, struct stat * st)
{
    (void)r;
    (void)file;

    memset(st, 0, sizeof(*st));
    st->st_mode = S_IFCHR;
    return 0;
}

int _isatty_r(struct _reent *r, int fd)
{
    (void)r;
    (void)fd;

    return 1;
}

/* Called by __libc_fini_array() */
void _fini(void)
{
    /* empty */
}
