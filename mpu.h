/*
 * Copyright (c) 2016, Antoine Albertelli
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this
 * list of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
*/
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
 */
void mpu_configure_region(int region, void *addr, size_t log2_len,
                          access_permission_t ap, bool executable);


#ifdef __cplusplus
}
#endif

#endif
