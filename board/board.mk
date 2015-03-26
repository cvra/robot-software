# List of all the board related files.
BOARDSRC = board/board.c

# Required include directories
BOARDINC = board/

ifeq ($(USE_BOOTLOADER), yes)
  BOARDDEFS = -DCORTEX_VTOR_INIT=0x08003800
  BOARDLDSCRIPT = board/board_with_bootloader.ld
else
  BOARDLDSCRIPT = board/board.ld
endif