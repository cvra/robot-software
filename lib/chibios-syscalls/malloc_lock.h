#ifndef MALLOC_LOCK_H
#define MALLOC_LOCK_H

#ifdef __cplusplus
extern "C" {
#endif

/** Creates the malloc mutex.
 *
 * @note Must be called after chSysInit but before using malloc in various
 * threads.
 */
void malloc_lock_init(void);

#ifdef __cplusplus
}
#endif

#endif /* MALLOC_LOCK_H */
