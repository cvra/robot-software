#include <string.h>
#include <stdio.h>
#include <math.h>
#include <stdint.h>
#include <hal.h>
#include <arm-cortex-tools/mpu.h>

void mpu_enable(void)
{
    MPU->CTRL |= MPU_CTRL_ENABLE_Msk;
}

void mpu_disable(void)
{
    MPU->CTRL &= ~MPU_CTRL_ENABLE_Msk;
}

void mpu_configure_region(int region, void* addr, size_t len, access_permission_t ap, bool executable)
{
    int32_t rasr = 0;

    chDbgAssert(len >= 5, "region too small");

    /* Extract region and address information. */
    region = region & MPU_RBAR_REGION_Msk;
    addr = (void*)((uint32_t)addr & MPU_RBAR_ADDR_Msk);

    /* If the region is not executable add the eXecute Never flag. */
    if (!executable) {
        rasr += MPU_RASR_XN_Msk;
    }

    /* Construct the Region Attributes and Size Register value. */
    rasr += (ap << MPU_RASR_AP_Pos);
    rasr += ((len - 1) << MPU_RASR_SIZE_Pos);
    rasr += MPU_RASR_ENABLE_Msk;

    /* Update the MPU settings. */
    MPU->RBAR = (uintptr_t)addr + region + MPU_RBAR_VALID_Msk;
    MPU->RASR = rasr;

    /* Make sure the memory barriers are correct. */
    __ISB();
    __DSB();
}

void mpu_init(void)
{
    /* Enable default memory permissions for priviledged code. */
    MPU->CTRL |= MPU_CTRL_PRIVDEFENA_Msk;

    /* Enable MemManage faults. */
    SCB->SHCSR |= SCB_SHCSR_MEMFAULTENA_Msk;

    /* NULL pointer protection, highest priority. */
    mpu_configure_region(7, NULL, 5, AP_NO_NO, false);

    /* Mark RAM as non executable. Using lowest priority means explicitely
     * allowing a RAM region to be executable is possible.
     *
     * We can use a fixed address range for RAM because it is a standard define
     * by ARM. */
    void* ram_base = (void*)0x20000000;
    int ram_size_pow2 = 29; /* 2^29 = 0.5GB. */
    mpu_configure_region(0, ram_base, ram_size_pow2, AP_RW_RW, false);

    mpu_enable();
}
