# List of all the board related files.
BOARDSRC = board/board.c

# Required include directories
BOARDINC = board/

ifeq ($(USE_BOOTLOADER), yes)
  BOARDDEFS = -DCORTEX_VTOR_INIT=0x08003800
  BOARDDEFS += -DCONFIG_ADDR=0x08002800
  BOARDDEFS += -DCONFIG_PAGE_SIZE=2048
  BOARDLDSCRIPT = ../linker/STM32F303xC_bootloader.ld
else
  BOARDLDSCRIPT = ../linker/STM32F303xC.ld
endif
