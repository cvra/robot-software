/*
 * STM32F303xC memory setup.
 * Bootloader & config pages: 0x08000000, len = 14K
 * -> Application: 0x08003800, len = 242K   // 256K - 14K
 */
MEMORY
{
    flash : org = 0x08003800, len = 242k
    ram : org = 0x20000000, len = 40k
    ccmram : org = 0x10000000, len = 8k
}

INCLUDE ChibiOS/os/common/ports/ARMCMx/compilers/GCC/rules.ld
