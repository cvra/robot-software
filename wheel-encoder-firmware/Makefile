##############################################################################
# Build global options
# NOTE: Can be overridden externally.
#

# Compiler options here.
ifeq ($(USE_OPT),)
  USE_OPT = -O2 -ggdb -fomit-frame-pointer -falign-functions=16
  USE_OPT += -fno-stack-protector -ftree-loop-distribute-patterns
  USE_OPT += -frename-registers -freorder-blocks -fconserve-stack
endif

# C specific options here (added to USE_OPT).
ifeq ($(USE_COPT),)
  USE_COPT =
endif

# C++ specific options here (added to USE_OPT).
ifeq ($(USE_CPPOPT),)
  USE_CPPOPT = -fno-rtti
  USE_CPPOPT +=  -std=gnu++11
  USE_CPPOPT += -fno-exceptions -fno-unwind-tables -fno-threadsafe-statics
endif

# Enable this if you want the linker to remove unused code and data
ifeq ($(USE_LINK_GC),)
  USE_LINK_GC = yes
endif

# Linker extra options here.
ifeq ($(USE_LDOPT),)
  USE_LDOPT =
endif

# Enable this if you want link time optimizations (LTO)
ifeq ($(USE_LTO),)
  USE_LTO = no
endif

# If enabled, this option allows to compile the application in THUMB mode.
ifeq ($(USE_THUMB),)
  USE_THUMB = yes
endif

# Enable this if you want to see the full log while compiling.
ifeq ($(USE_VERBOSE_COMPILE),)
  USE_VERBOSE_COMPILE = no
endif

#
# Build global options
##############################################################################

##############################################################################
# Architecture or project specific options
#

# Stack size to be allocated to the Cortex-M process stack. This stack is
# the stack used by the main() thread.
ifeq ($(USE_PROCESS_STACKSIZE),)
  USE_PROCESS_STACKSIZE = 0x400
endif

# Stack size to the allocated to the Cortex-M main/exceptions stack. This
# stack is used for processing interrupts and exceptions.
ifeq ($(USE_EXCEPTIONS_STACKSIZE),)
  USE_EXCEPTIONS_STACKSIZE = 0x400
endif

# Enables the use of FPU on Cortex-M4 (no, softfp, hard).
ifeq ($(USE_FPU),)
  USE_FPU = hard
endif

#
# Architecture or project specific options
##############################################################################

##############################################################################
# Project, sources and paths
#

# Define project name here
PROJECT = ch

# Imported source files and paths
CHIBIOS = ../lib/ChibiOS
include $(CHIBIOS)/os/hal/hal.mk
include $(CHIBIOS)/os/common/startup/ARMCMx/compilers/GCC/mk/startup_stm32f3xx.mk
include $(CHIBIOS)/os/hal/ports/STM32/STM32F3xx/platform.mk
include $(CHIBIOS)/os/hal/osal/rt/osal.mk
include $(CHIBIOS)/os/rt/rt.mk
include $(CHIBIOS)/os/common/ports/ARMCMx/compilers/GCC/mk/port_v7m.mk
include $(CHIBIOS)/os/hal/ports/STM32/STM32F3xx/platform.mk
# include $(CHIBIOS)/test/rt/test.mk

ifeq ($(USE_BOOTLOADER),)
  USE_BOOTLOADER = yes
endif

# Define linker script file here
ifeq ($(USE_BOOTLOADER), yes)
  UDEFS = -DCORTEX_VTOR_INIT=0x08003800
  UDEFS += -DCONFIG_ADDR=0x08002800
  UDEFS += -DCONFIG_PAGE_SIZE=2048
  LDSCRIPT = linker/STM32F303xC_bootloader.ld
else
  LDSCRIPT = linker/STM32F303xC.ld
endif

include src/src.mk

# C sources that can be compiled in ARM or THUMB mode depending on the global
# setting.
CSRC = $(PORTSRC) \
       $(KERNSRC) \
       $(TESTSRC) \
       $(HALSRC) \
       $(OSALSRC) \
       $(PLATFORMSRC) \
       $(STARTUPSRC) \
       $(CHIBIOS)/os/hal/lib/streams/chprintf.c \
       $(CHIBIOS)/os/hal/lib/streams/memstreams.c \
       src/version.c \
       $(PROJCSRC)

# C++ sources that can be compiled in ARM or THUMB mode depending on the global
# setting.
CPPSRC = $(CHCPPSRC) $(PROJCPPSRC)

# C sources to be compiled in ARM mode regardless of the global setting.
# NOTE: Mixing ARM and THUMB mode enables the -mthumb-interwork compiler
#       option that results in lower performance and larger code size.
ACSRC =

# C++ sources to be compiled in ARM mode regardless of the global setting.
# NOTE: Mixing ARM and THUMB mode enables the -mthumb-interwork compiler
#       option that results in lower performance and larger code size.
ACPPSRC =

# C sources to be compiled in THUMB mode regardless of the global setting.
# NOTE: Mixing ARM and THUMB mode enables the -mthumb-interwork compiler
#       option that results in lower performance and larger code size.
TCSRC =

# C sources to be compiled in THUMB mode regardless of the global setting.
# NOTE: Mixing ARM and THUMB mode enables the -mthumb-interwork compiler
#       option that results in lower performance and larger code size.
TCPPSRC =

# List ASM source files here
ASMSRC =
ASMXSRC = $(STARTUPASM) $(PORTASM) $(OSALASM)

INCDIR = $(PORTINC) $(KERNINC) $(TESTINC) \
         $(HALINC) $(OSALINC) $(PLATFORMINC) $(STARTUPINC) ./board \
         $(CHIBIOS)/os/various $(CHIBIOS)/os/hal/lib/streams ./src \
         $(CHCPPINC) $(CHIBIOS)/os/license

# ChibiOS C++ bindings
CPPSRC += $(CHIBIOS)/os/various/cpp_wrappers/ch.cpp
INCDIR += $(CHIBIOS)/os/various/cpp_wrappers

#
# Project, sources and paths
##############################################################################

##############################################################################
# Compiler settings
#

MCU  = cortex-m4

#TRGT = arm-elf-
TRGT = arm-none-eabi-
CC   = $(TRGT)gcc
CPPC = $(TRGT)g++
# Enable loading with g++ only if you need C++ runtime support.
# NOTE: You can use C++ even without C++ support if you are careful. C++
#       runtime support makes code size explode.
# LD   = $(TRGT)gcc
LD   = $(TRGT)g++
CP   = $(TRGT)objcopy
AS   = $(TRGT)gcc -x assembler-with-cpp
AR   = $(TRGT)ar
OD   = $(TRGT)objdump
SZ   = $(TRGT)size
HEX  = $(CP) -O ihex
BIN  = $(CP) -O binary

# ARM-specific options here
AOPT =

# THUMB-specific options here
TOPT = -mthumb -DTHUMB

# Define C warning options here
CWARN = -Wall -Wextra -Wstrict-prototypes

# Define C++ warning options here
CPPWARN = -Wall -Wextra

#
# Compiler settings
##############################################################################

##############################################################################
# Start of user section
#

# List all user C define here, like -D_DEBUG=1
UDEFS += -DUAVCAN_STM32_CHIBIOS=1 \
		 -DUAVCAN_TOSTRING=0 \
		 -DUAVCAN_DEBUG=0 \
		 -DUAVCAN_STM32_TIMER_NUMBER=2 \
		 -DUAVCAN_STM32_NUM_IFACES=1

# Define ASM defines here
UADEFS =

# List all user directories here
UINCDIR = $(PROJINC) ./src

# List the user directory to look for the libraries here
ULIBDIR =

# List all user libraries here
ULIBS =

#
# UAVCAN
#
include ../lib/uavcan/libuavcan/include.mk
include ../lib/uavcan/libuavcan_drivers/stm32/driver/include.mk

CPPSRC += $(LIBUAVCAN_SRC) $(LIBUAVCAN_STM32_SRC)
UINCDIR += $(LIBUAVCAN_INC) $(LIBUAVCAN_STM32_INC) ./dsdlc_generated

#
# End of user defines
##############################################################################

RULESPATH = $(CHIBIOS)/os/common/startup/ARMCMx/compilers/GCC
include $(RULESPATH)/rules.mk

# run uavcan dsdl compiler
.PHONY: dsdlc
dsdlc:
	@$(COLOR_PRINTF) "Running uavcan dsdl compiler"
	$(LIBUAVCAN_DSDLC) ../uavcan_data_types/cvra $(UAVCAN_DSDL_DIR)

PRE_MAKE_ALL_RULE_HOOK: src/version.c
.PHONY: src/version.c

src/version.c:
	@../lib/version/version.sh src/version.c

.PHONY: flash
flash: all
	openocd -f openocd.cfg -c "program $(BUILDDIR)/$(PROJECT).elf verify reset" -c "shutdown"

.PHONY: reset
reset:
	openocd -f openocd.cfg -c "init" -c "reset" -c "shutdown"

include ../tools/generate_ctags.mk
