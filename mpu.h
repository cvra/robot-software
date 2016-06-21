#ifndef MEMORY_PROTECTION_H
#define MEMORY_PROTECTION_H

#ifdef __cplusplus
extern "C" {
#endif

/** Memory access permissions.
 *
 * The first group is for priviledged code, the second for unpriviledged code.
 */
typedef enum {
    AP_NO_NO=0x0,
    AP_RW_NO=0x1,
    AP_RW_RO=0x2,
    AP_RW_RW=0x3,
    AP_RESERVED=0x4,
    AP_RO_NO=0x5,
    AP_RO_RO=0x6,
} access_permission_t;

/** Initializes the Memory Protection Unit.
 *
 * @note This function also registers a region preventing access to NULL and
 * having max priority (region 7).
 * @note This function enables the MPU.
 */
void mpu_init(void);

/** Enables memory protection. */
void mpu_enable(void);

/** Disables memory protection. */
void mpu_disable(void);

/** Configures an MPU region.
 *
 * @param [in] region Region number (0-7)
 * @param [in] addr Base address of the memory region.
 * @param [in] log2_len Log in base 2 of the length. Ex: 32B -> 5, 1K -> 10.
 * @param [in] ap Access permission of the region.
 * @param [in] executable True if code contained in the code should be
 * executable.
 *
 * @note The lowest 5 bits of the base address are ignored.
 * @warning This function makes the assumption that the region is in memory,
 * not peripheral space (B flag=0).
 * @warning This function is not thread-safe.
 */
void mpu_configure_region(int region, void *addr, size_t log2_len,
                          access_permission_t ap, bool executable);


#ifdef __cplusplus
}
#endif

#endif
