BOARDINC = src/board/ST_NUCLEO144_F429ZI
BOARDSRC = $(BOARDINC)/board.c

ifeq ($(USE_BOOTLOADER), yes)
  $(error bootloader for ST_NUCLEO144_F429ZI not supported)
else
  LDSCRIPT= linker/STM32F429xI.ld
endif
