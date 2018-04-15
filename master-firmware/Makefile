##############################################################################
# Build global options
# NOTE: Can be overridden externally.
#

# Compiler options here.
ifeq ($(USE_OPT),)
  USE_OPT = -O2 -ggdb -fomit-frame-pointer -falign-functions=16
  USE_OPT += -frename-registers -freorder-blocks -fconserve-stack -fstack-usage
  USE_OPT += -fstack-protector-all -L .
endif

# C specific options here (added to USE_OPT).
ifeq ($(USE_COPT),)
  USE_COPT = -std=gnu99
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


# Config file depending on robot
ifeq ($(ROBOT), )
  $(error Please define ROBOT to either 'order' or 'chaos')
endif

ifeq ($(ROBOT), order)
  CONFIG_YAML = ../config_order.yaml
else
  CONFIG_YAML = ../config_chaos.yaml
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
CHIBIOS = ../lib/ChibiOS

# Startup files.
include $(CHIBIOS)/os/common/startup/ARMCMx/compilers/GCC/mk/startup_stm32f4xx.mk

# HAL-OSAL files (optional).
include $(CHIBIOS)/os/hal/hal.mk
include $(CHIBIOS)/os/hal/ports/STM32/STM32F4xx/platform.mk
include $(CHIBIOS)/os/hal/osal/rt/osal.mk

# RTOS files (optional).
include $(CHIBIOS)/os/rt/rt.mk
include $(CHIBIOS)/os/common/ports/ARMCMx/compilers/GCC/mk/port_v7m.mk

# Other files (optional).
#include $(CHIBIOS)/test/rt/test.mk

# ChibiOS C++ bindings
include $(CHIBIOS)/os/various/cpp_wrappers/chcpp.mk

# Target board
include src/board/board.mk

# Shell module
include $(CHIBIOS)/os/various/shell/shell.mk

# uGFX (GUI library)
GFXLIB=../lib/ugfx
OPT_OS=chibios
include $(GFXLIB)/gfx.mk


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
	   $(SHELLSRC) \
	   $(CHIBIOS)/os/various/evtimer.c \
	   $(CHIBIOS)/os/hal/lib/streams/memstreams.c \
	   $(CHIBIOS)/os/hal/lib/streams/chprintf.c \
	   $(CHIBIOS)/os/various/syscalls.c \
	   $(CHIBIOS)/os/various/fatfs_bindings/fatfs_diskio.c \
	   $(BOARDSRC) \
	   $(GFXSRC)

include app_src.mk

# C++ sources that can be compiled in ARM or THUMB mode depending on the global
# setting.
CPPSRC += $(CHCPPSRC)

# List ASM source files here
ASMSRC +=
ASMXSRC +=  $(STARTUPASM) $(PORTASM) $(OSALASM)

INCDIR += $(CHIBIOS)/os/license \
		  $(STARTUPINC) $(KERNINC) $(PORTINC) $(OSALINC) \
		  $(HALINC) $(PLATFORMINC) $(LWINC) \
		  $(CHCPPINC) \
		  $(CHIBIOS)/os/various \
		  $(CHIBIOS)/os/hal/lib/streams \
		  $(SHELLINC) \
		  $(TESTINC) \
		  $(BOARDINC) \
		  $(GFXINC)

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

#
# Compiler settings
##############################################################################

##############################################################################
# Start of default section
#

# List all default C defines here, like -DDEBUG
DDEFS += -DUAVCAN_TOSTRING=0 \
		 -DUAVCAN_STM32_NUM_IFACES=1 \
		 -DUAVCAN_STM32_TIMER_NUMBER=2 \
		 -DUAVCAN_STM32_CHIBIOS=1 \
		 -DUAVCAN_TINY=0 \
		 -DLWIP_DEBUG=1 \
		 -DSHELL_CONFIG_FILE

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
UDEFS = -DON_ROBOT
UDEFS += $(addprefix -D, $(GFXDEFS))

ifeq ($(ROBOT), order)
  UDEFS += -DDEBRA=1
endif

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
include ../lib/uavcan/libuavcan/include.mk
include ../lib/uavcan/libuavcan_drivers/stm32/driver/include.mk

CPPSRC += $(LIBUAVCAN_SRC) $(LIBUAVCAN_STM32_SRC)

UINCDIR += $(LIBUAVCAN_INC) ./dsdlc_generated $(LIBUAVCAN_STM32_INC)

#
# End of user defines
##############################################################################

RULESPATH = ./linker/
include $(CHIBIOS)/os/common/startup/ARMCMx/compilers/GCC/rules.mk

# Empty libraries, required by stack smashing protection
PRE_MAKE_ALL_RULE_HOOK: app_src.mk libssp.a libssp_nonshared.a src/config_msgpack.c
libssp.a:
	arm-none-eabi-ar rcs $@

libssp_nonshared.a:
	arm-none-eabi-ar rcs $@

.PHONY: mem_info
mem_info: $(BUILDDIR)/$(PROJECT).elf
	arm-none-eabi-nm --size-sort --print-size $(BUILDDIR)/$(PROJECT).elf > $(BUILDDIR)/memory_size.txt
	arm-none-eabi-nm --numeric-sort --print-size $(BUILDDIR)/$(PROJECT).elf > $(BUILDDIR)/memory_layout.txt

# Openocd config file
ifeq ($(OPENOCD_CFG), )
  OPENOCD_CFG = oocd_stlink.cfg
endif

.PHONY: flash
flash: $(BUILDDIR)/$(PROJECT).elf
	openocd -f $(OPENOCD_CFG) -c "program $(BUILDDIR)/$(PROJECT).elf verify reset" -c "shutdown"
	say -v "Daniel" "Robot flashed master!"

# run uavcan dsdl compiler
.PHONY: dsdlc
dsdlc:
	@$(COLOR_PRINTF) "Running uavcan dsdl compiler"
	$(LIBUAVCAN_DSDLC) ../uavcan_data_types/cvra $(UAVCAN_DSDL_DIR)

src/config_msgpack.c: ${CONFIG_YAML}
	@$(COLOR_PRINTF) "Generating MessagePack config..."
	@../tools/config/config_to_msgpack.py ${CONFIG_YAML} src/config_msgpack.c

.PHONY: heap_size
heap_size: $(BUILDDIR)/$(PROJECT).elf
	@../tools/heap_size.py $(BUILDDIR)/$(PROJECT).elf

all: heap_size mem_info

CLEAN_RULE_HOOK:
	@$(COLOR_PRINTF) "Remove MessagePack config"
	-rm -f src/config_msgpack.c

include ../tools/generate_ctags.mk
