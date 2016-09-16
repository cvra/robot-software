BOARDINC = src/board/ST_NUCLEO144_F429ZI
BOARDSRC = $(BOARDINC)/board.c

ifeq ($(USE_BOOTLOADER), yes)
  # todo
else
  LDSCRIPT= STM32F429xI.ld
endif
