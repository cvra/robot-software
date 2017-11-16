#ifndef PARAMETER_PORT_H
#define PARAMETER_PORT_H

#ifdef __cplusplus
extern "C" {
#endif

/** Acquires a mutual exclusion lock on the parameter tree. */
extern void parameter_port_lock(void);

/** Releases the lock acquired by parameter_port_lock. */
extern void parameter_port_unlock(void);

/** Checks that condition is true and aborts execution otherwise. */
extern void parameter_port_assert(int condition);

/** Allocates a buffer of at least the given size.
 *
 * @note This is only used when converting MessagePack objects.
 * @note At most one buffer is allocated at any moment, which allows the use of
 * a static buffer.
 * @note The maximum allocated size is the length of the longest parameter or
 * namespace name or the largest parameter array, matrix or string, whichever
 * is the largest.
 */
extern void *parameter_port_buffer_alloc(size_t size);

/** Frees a buffer allocated by parameter_port_buffer_alloc. */
extern void parameter_port_buffer_free(void *buffer);

#ifdef __cplusplus
}
#endif

#endif /* PARAMETER_PORT_H */

