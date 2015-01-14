# Compiler options here.
USE_OPT = -O2 -ggdb -fomit-frame-pointer -falign-functions=16
USE_CPPOPT = -fno-rtti
USE_LDOPT = 

# Remove unuused functions from binary
USE_LINK_GC = yes
USE_LTO = no
USE_THUMB = yes

# Enable this if you want to see the full log while compiling.
USE_VERBOSE_COMPILE = no

# Enables the use of FPU on Cortex-M4 (no, softfp, hard).
USE_FPU = hard

# Define project name here
PROJECT = cvra-master

# Imported source files and paths
CHIBIOS = ChibiOS
include $(CHIBIOS)/boards/OLIMEX_STM32_E407_REV_D/board.mk
include $(CHIBIOS)/os/hal/platforms/STM32F4xx/platform.mk
include $(CHIBIOS)/os/hal/hal.mk
include $(CHIBIOS)/os/ports/GCC/ARMCMx/STM32F4xx/port.mk
include $(CHIBIOS)/os/kernel/kernel.mk
include lwip.mk
include $(CHIBIOS)/test/test.mk

# Define linker script file here
LDSCRIPT= $(PORTLD)/STM32F407xG.ld

# C sources that can be compiled in ARM or THUMB mode depending on the global
# setting.
CSRC = $(PORTSRC) \
       $(KERNSRC) \
       $(TESTSRC) \
       $(HALSRC) \
       $(PLATFORMSRC) \
       $(BOARDSRC) \
       $(LWSRC) \
       $(CHIBIOS)/os/various/evtimer.c \
       $(CHIBIOS)/os/various/chprintf.c \
       $(CHIBIOS)/os/various/shell.c

# C++ sources that can be compiled in ARM or THUMB mode depending on the global
# setting.
CPPSRC =

# List ASM source files here
ASMSRC = $(PORTASM)

INCDIR = $(PORTINC) $(KERNINC) $(TESTINC) \
         $(HALINC) $(PLATFORMINC) $(BOARDINC) $(LWINC) \
         $(CHIBIOS)/os/various

# Application source, managed by packager.py
include app_src.mk

# Compiler settings
TRGT = arm-none-eabi-
MCU  = cortex-m4
CC   = $(TRGT)gcc
CPPC = $(TRGT)g++
LD   = $(TRGT)g++
CP   = $(TRGT)objcopy
AS   = $(TRGT)gcc -x assembler-with-cpp
OD   = $(TRGT)objdump
SZ   = $(TRGT)size
HEX  = $(CP) -O ihex
BIN  = $(CP) -O binary

# THUMB-specific options here
TOPT = -mthumb -DTHUMB

# Define C warning options here
CWARN = -Wall -Wextra -Wstrict-prototypes

# Define C++ warning options here
CPPWARN = -Wall -Wextra

# List all C defines here, like -D_DEBUG=1
DDEFS =

# List all ASM defines here, like -D_DEBUG=1
DADEFS =

# List all directories to look for include files here
DINCDIR =

# List the directory to look for the libraries here
DLIBDIR =

# List all libraries here
DLIBS =

RULESPATH = $(CHIBIOS)/os/ports/GCC/ARMCMx
include $(RULESPATH)/rules.mk

flash: all
	openocd -f oocd.cfg -c "program build/$(PROJECT).elf verify reset"
