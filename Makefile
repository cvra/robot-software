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
  USE_CPPOPT += -std=gnu++11
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
CHIBIOS = ChibiOS

# Startup files.
include $(CHIBIOS)/os/common/ports/ARMCMx/compilers/GCC/mk/startup_stm32f4xx.mk

# HAL-OSAL files (optional).
include $(CHIBIOS)/os/hal/hal.mk
include $(CHIBIOS)/os/hal/ports/STM32/STM32F4xx/platform.mk
include $(CHIBIOS)/os/hal/osal/rt/osal.mk

# RTOS files (optional).
include $(CHIBIOS)/os/rt/rt.mk
include $(CHIBIOS)/os/rt/ports/ARMCMx/compilers/GCC/mk/port_v7m.mk

# Other files (optional).
include $(CHIBIOS)/test/rt/test.mk

# lwIP files
include lwip.mk

# ChibiOS C++ bindings
include $(CHIBIOS)/os/various/cpp_wrappers/chcpp.mk

# Bootloader settings
USE_BOOTLOADER = no

ifeq ($(USE_BOOTLOADER), yes)
  DDEFS += -DCORTEX_VTOR_INIT=0x0800C000
  LDSCRIPT= STM32F407xG_bootloader.ld
else
  LDSCRIPT= STM32F407xG.ld
endif

# C sources that can be compiled in ARM or THUMB mode depending on the global
# setting.
CSRC = $(PORTSRC) \
       $(KERNSRC) \
       $(TESTSRC) \
       $(HALSRC) \
       $(PLATFORMSRC) \
	   $(STARTUPSRC) \
       $(LWSRC) \
       $(FATFSSRC) \
       $(CHIBIOS)/os/various/evtimer.c \
       $(CHIBIOS)/os/hal/lib/streams/memstreams.c \
       $(CHIBIOS)/os/hal/lib/streams/chprintf.c \
       $(CHIBIOS)/os/various/syscalls.c \
       $(CHIBIOS)/os/various/shell.c

include app_src.mk

# C++ sources that can be compiled in ARM or THUMB mode depending on the global
# setting.
CPPSRC += $(CHCPPSRC)

# List ASM source files here
ASMSRC = $(STARTUPASM) $(PORTASM) $(OSALASM)

INCDIR += $(PORTINC) $(KERNINC) $(TESTINC) \
		  $(STARTUPINC) \
          $(HALINC) $(PLATFORMINC) $(LWINC) \
          $(OSALINC) \
          $(CHCPPINC) \
          $(CHIBIOS)/os/various \
		  $(CHIBIOS)/os/hal/lib/streams

#
# Project, sources and paths
##############################################################################

##############################################################################
# Compiler settings
#

MCU  = cortex-m4

TRGT = arm-none-eabi-
CC   = $(TRGT)gcc
CPPC = $(TRGT)g++
# Enable loading with g++ only if you need C++ runtime support.
# NOTE: You can use C++ even without C++ support if you are careful. C++
#       runtime support makes code size explode.
# LD   = $(TRGT)gcc
LD   = $(TRGT)g++
CP   = $(TRGT)objcopy -j startup -j constructors -j destructors -j .text \
		-j .ARM.extab -j .ARM.exidx -j .eh_frame_hdr -j .eh_frame -j .textalign -j .data
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

#
# Compiler settings
##############################################################################

##############################################################################
# Start of default section
#

# List all default C defines here, like -DDEBUG
DDEFS += -DUAVCAN_TOSTRING=0 \
		 -DUAVCAN_STM32_NUM_IFACES=1 \
		 -DUAVCAN_STM32_TIMER_NUMBER=3 \
		 -DUAVCAN_STM32_CHIBIOS=1 \
		 -DUAVCAN_TINY=0 \
		 -DLWIP_DEBUG=1

# List all default ASM defines here, like -D_DEBUG=1
DADEFS =

# List all default directories to look for include files here
DINCDIR =

# List the default directory to look for the libraries here
DLIBDIR =

# List all default libraries here
DLIBS =

#
# End of default section
##############################################################################

##############################################################################
# Start of user section
#

# List all user C define here, like -D_DEBUG=1
UDEFS = -DDEBRA=1 -DON_ROBOT

# Define ASM defines here
UADEFS =

# List all user directories here
UINCDIR =

# List the user directory to look for the libraries here
UINCDIR =

# List all user libraries here
ULIBS =

#
# UAVCAN
##############################################################################
include uavcan/libuavcan/include.mk
include uavcan/libuavcan_drivers/stm32/driver/include.mk

CPPSRC += $(LIBUAVCAN_SRC) $(LIBUAVCAN_STM32_SRC)

UINCDIR += $(LIBUAVCAN_INC) ./dsdlc_generated $(LIBUAVCAN_STM32_INC)

#
# End of user defines
##############################################################################

GLOBAL_SRC_DEP = app_src.mk

RULESPATH = $(CHIBIOS)/os/common/ports/ARMCMx/compilers/GCC
include $(RULESPATH)/rules.mk

.PHONY: mem_info
mem_info: build/$(PROJECT).elf
	arm-none-eabi-nm --size-sort --print-size build/$(PROJECT).elf > build/memory_size.txt
	arm-none-eabi-nm --numeric-sort --print-size build/$(PROJECT).elf > build/memory_layout.txt

.PHONY: flash
flash: build/$(PROJECT).elf
	openocd -f oocd.cfg -c "program build/ch.elf verify reset" -c "shutdown"

# run uavcan dsdl compiler
.PHONY: dsdlc
dsdlc:
	@$(COLOR_PRINTF) "Running uavcan dsdl compiler"
	$(LIBUAVCAN_DSDLC) cvra $(UAVCAN_DSDL_DIR)

.PHONY: heap_size
heap_size: build/$(PROJECT).elf
	@python/heap_size.py build/$(PROJECT).elf

all: heap_size mem_info

# Generates a ctags file containing the correct definition for the build
.PHONY: ctags
ctags:
	@echo "Generating ctags file..."
	@cat .dep/*.d | grep ":$$" | sed "s/://" | sort | uniq | xargs ctags --file-scope=no --extra=+q $(CSRC) $(CPPSRC)
