#ifndef PARAMETER_FLASH_STORAGE_H
#define PARAMETER_FLASH_STORAGE_H

#ifdef __cplusplus
extern "C" {
#endif

#include <parameter/parameter.h>

#define PARAMETER_FLASH_STORAGE_HEADER_SIZE (3 * sizeof(uint32_t))

/** Erase config sector.
 */
void parameter_flash_storage_erase(void *dst);

/** Writes the given parameter namespace to flash , prepending it with a CRC
 * for integrity checks.
 */
void parameter_flash_storage_save(void *dst, size_t dst_len, parameter_namespace_t *ns);

/** Loads the configuration from the flash
 *
 * @returns true if the operation was successful.
 * @note If no valid block is found the parameter tree is unchanged.
 */
bool parameter_flash_storage_load(parameter_namespace_t *ns, void *src);
#ifdef __cplusplus
}
#endif


#endif
