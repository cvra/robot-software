/*
 * STM32F303xC memory setup.
 * to use with bootloader change the LDSCRIPT to board_with_bootloader.ld in the Makefile
 */
MEMORY
{
    flash : org = 0x08000000, len = 256K
    ram0  : org = 0x20000000, len = 40k
    ram1  : org = 0x00000000, len = 0
    ram2  : org = 0x00000000, len = 0
    ram3  : org = 0x00000000, len = 0
    ram4  : org = 0x10000000, len = 8k
    ram5  : org = 0x00000000, len = 0
    ram6  : org = 0x00000000, len = 0
    ram7  : org = 0x00000000, len = 0
}

/* RAM region to be used for Main stack. This stack accommodates the processing
   of all exceptions and interrupts*/
REGION_ALIAS("MAIN_STACK_RAM", ram0);

/* RAM region to be used for the process stack. This is the stack used by
   the main() function.*/
REGION_ALIAS("PROCESS_STACK_RAM", ram0);

/* RAM region to be used for data segment.*/
REGION_ALIAS("DATA_RAM", ram0);

/* RAM region to be used for BSS segment.*/
REGION_ALIAS("BSS_RAM", ram0);

INCLUDE ../lib/ChibiOS/os/common/ports/ARMCMx/compilers/GCC/rules.ld
