#ifndef PANIC_LOG_H_
#define PANIC_LOG_H_

/** Writes the given message to the kernel panic log. */
void panic_log_write(const char *msg);

/** Returns a string read from the panic log or NULL if it is empty
 *
 * @note This string is read directly from the buffer, so any call to
 * panic_log_write will corrupt it.
 */
const char *panic_log_read(void);

/** Clears the panic log.
 *
 * @note Any further call to panic_log_read() will return NULL.
 */
void panic_log_clear(void);

#endif
