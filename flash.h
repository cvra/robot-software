#ifndef FLASH_H
#define FLASH_H

#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

void flash_lock(void);
void flash_unlock(void);

/* Write data of size len at addr.
 * Note: flash must be unlocket for this operation. */
void flash_write(void *addr, const void *data, size_t len);

/** Erase sector given by its base address.
 *
 * @note flash must be unlocked for this operation.
 */
void flash_sector_erase(void *addr);

uint8_t flash_addr_to_sector(void *p);
void flash_sector_erase_number(uint8_t sector);

#ifdef __cplusplus
}
#endif

#endif /* FLASH_H */
