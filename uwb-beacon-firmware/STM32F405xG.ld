/*
    ChibiOS - Copyright (C) 2006..2015 Giovanni Di Sirio.

    This file is part of ChibiOS.

    ChibiOS is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 3 of the License, or
    (at your option) any later version.

    ChibiOS is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

/*
 * STM32F405xG memory setup.
 */
MEMORY
{
    flash : org = 0x08000000, len = 896k
    config : org = 0x080e0000, len = 128k   /* Last sector hold configuration data */
    ram0  : org = 0x20000000, len = 128k    /* SRAM1 + SRAM2 */
    ram1  : org = 0x20000000, len = 112k    /* SRAM1 */
    ram2  : org = 0x2001C000, len = 16k     /* SRAM2 */
    ram3  : org = 0x00000000, len = 0
    ram4  : org = 0x10000000, len = 64k     /* CCM SRAM */
    ram5  : org = 0x40024000, len = 4k      /* BCKP SRAM */
    ram6  : org = 0x00000000, len = 0
    ram7  : org = 0x00000000, len = 0
}

/* For each data/text section two region are defined, a virtual region
   and a load region (_LMA suffix).*/

/* Flash region to be used for exception vectors.*/
REGION_ALIAS("VECTORS_FLASH", flash);
REGION_ALIAS("VECTORS_FLASH_LMA", flash);

/* Flash region to be used for constructors and destructors.*/
REGION_ALIAS("XTORS_FLASH", flash);
REGION_ALIAS("XTORS_FLASH_LMA", flash);

/* Flash region to be used for code text.*/
REGION_ALIAS("TEXT_FLASH", flash);
REGION_ALIAS("TEXT_FLASH_LMA", flash);

/* Flash region to be used for read only data.*/
REGION_ALIAS("RODATA_FLASH", flash);
REGION_ALIAS("RODATA_FLASH_LMA", flash);

/* Flash region to be used for various.*/
REGION_ALIAS("VARIOUS_FLASH", flash);
REGION_ALIAS("VARIOUS_FLASH_LMA", flash);

/* Flash region to be used for RAM(n) initialization data.*/
REGION_ALIAS("RAM_INIT_FLASH_LMA", flash);


/* RAM region to be used for Main stack. This stack accommodates the processing
   of all exceptions and interrupts*/
REGION_ALIAS("MAIN_STACK_RAM", ram0);

/* RAM region to be used for the process stack. This is the stack used by
   the main() function.*/
REGION_ALIAS("PROCESS_STACK_RAM", ram0);

/* RAM region to be used for data segment.*/
REGION_ALIAS("DATA_RAM", ram0);
REGION_ALIAS("DATA_RAM_LMA", flash);

/* RAM region to be used for BSS segment.*/
REGION_ALIAS("BSS_RAM", ram0);

/* RAM REGION to be usded for noinit section. */
REGION_ALIAS("NOINIT_RAM", ram0);

/* core coupled memory region */
REGION_ALIAS("CCM_RAM", ram4);

/* RAM region to be used for the default heap.*/
REGION_ALIAS("HEAP_RAM", ram0);

_config_start = ORIGIN(config);
_config_end = ORIGIN(config) + LENGTH(config);

__process_stack_size__ = 0x400;
__main_stack_size__ = 0x400;

INCLUDE rules.ld
