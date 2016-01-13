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
#include <string.h>
#include <stdio.h>
#include <math.h>
#include <stdint.h>
#include <hal.h>
#include "mpu.h"

void mpu_enable(void)
{
    MPU->CTRL |= MPU_CTRL_ENABLE_Msk;
}

void mpu_disable(void)
{
    MPU->CTRL &= ~MPU_CTRL_ENABLE_Msk;
}

void mpu_configure_region(int region, void *addr, size_t len,
                          access_permission_t ap, bool executable)
{
    int32_t rasr = 0;

    chDbgAssert(len >= 5, "region too small");

    /* Extract region and address information. */
    region = region & MPU_RBAR_REGION_Msk;
    addr = (void *)((uint32_t)addr & MPU_RBAR_ADDR_Msk);

    /* If the region is not executable add the eXecute Never flag. */
    if (!executable) {
        rasr += MPU_RASR_XN_Msk;
    }

    /* Construct the Region Attributes and Size Register value. */
    rasr += (ap << MPU_RASR_AP_Pos);
    rasr += ((len - 1) << MPU_RASR_SIZE_Pos);
    rasr += MPU_RASR_ENABLE_Msk;

    chSysLock();

    /* Enable MemManage faults. */
    SCB->SHCSR |= SCB_SHCSR_MEMFAULTENA_Msk;

    /* Update the MPU settings. */
    MPU->RBAR = (uintptr_t)addr + region + MPU_RBAR_VALID_Msk;
    MPU->RASR = rasr;

    chSysUnlock();

    /* Make sure the memory barriers are correct. */
    __ISB();
    __DSB();
}

void mpu_init(void)
{
    /* Enable default memory permissions for priviledged code. */
    MPU->CTRL |= MPU_CTRL_PRIVDEFENA_Msk;

    /* NULL pointer protection, highest priority. */
    mpu_configure_region(7, NULL, 5, AP_NO_NO, false);

    mpu_enable();
}

void MemManage_Handler(void)
{
    static char msg[128];
    struct port_extctx ctx;

    uint32_t MMFSR;

    /* Setup default error message. */
    strcpy(msg, __FUNCTION__);

    /* Get context info */
    memcpy(&ctx, (void*)__get_PSP(), sizeof(struct port_extctx));

    /* Get Memory Managment fault adress register */
    MMFSR = SCB->CFSR & SCB_CFSR_MEMFAULTSR_Msk;

    /* Data access violation */
    if (MMFSR & (1 << 1)) {
        snprintf(msg, sizeof(msg),
                 "Invalid access to %p (pc=%p)", (void *)SCB->MMFAR, ctx.pc);
    }

    /* Instruction address violation. */
    if (MMFSR & (1 << 0)) {
        snprintf(msg, sizeof(msg),
                 "Jumped to XN region %p (lr_thd=%p)",
                    (void *)SCB->MMFAR, ctx.lr_thd);
    }

    chSysHalt(msg);
}
