##############################################################################
# Build global options
# NOTE: Can be overridden externally.
#

# Compiler options here.
USE_OPT = -Os -ggdb -fomit-frame-pointer -falign-functions=16 -fno-common

# C specific options here (added to USE_OPT).
USE_COPT =

# C++ specific options here (added to USE_OPT).
USE_CPPOPT = -fno-rtti -fno-exceptions -fno-threadsafe-statics

# Enable this if you want the linker to remove unused code and data
USE_LINK_GC = yes

# Linker extra options here.
USE_LDOPT =

# Enable this if you want link time optimizations (LTO)
USE_LTO = no

# If enabled, this option allows to compile the application in THUMB mode.
USE_THUMB = yes

# Enable this if you want to see the full log while compiling.
USE_VERBOSE_COMPILE = no

# If enabled, this option makes the build process faster by not compiling
# modules not used in the current configuration.
USE_SMART_BUILD = no

#
# Build global options
##############################################################################

##############################################################################
# Architecture or project specific options
#

# Stack size to be allocated to the Cortex-M process stack. This stack is
# the stack used by the main() thread.
USE_PROCESS_STACKSIZE = 5000

# Stack size to the allocated to the Cortex-M main/exceptions stack. This
# stack is used for processing interrupts and exceptions.
USE_EXCEPTIONS_STACKSIZE = 0x400

# Enables the use of FPU on Cortex-M4 (no, softfp, hard).
USE_FPU = no

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
include $(CHIBIOS)/os/common/ports/ARMCMx/compilers/GCC/mk/startup_stm32f3xx.mk
# HAL-OSAL files (optional).
include $(CHIBIOS)/os/hal/hal.mk
include $(CHIBIOS)/os/hal/ports/STM32/STM32F3xx/platform.mk
include $(CHIBIOS)/os/hal/osal/rt/osal.mk
# RTOS files (optional).
include $(CHIBIOS)/os/rt/rt.mk
include $(CHIBIOS)/os/rt/ports/ARMCMx/compilers/GCC/mk/port_v7m.mk
# Other files (optional).
#include $(CHIBIOS)/test/rt/test.mk

# Define linker script file here

ifeq ($(USE_BOOTLOADER), )
	USE_BOOTLOADER = no
endif

ifeq ($(USE_BOOTLOADER), yes)
  DDEFS += -DCORTEX_VTOR_INIT=0x08003800
  LDSCRIPT= linker/STM32F302x8_bootloader.ld
  DDEFS += -DOCNFIG_ADDR=0x08002800 -DCONFIG_PAGE_SIZE=0x800
else
  LDSCRIPT= linker/STM32F302x8.ld
endif

# C sources that can be compiled in ARM or THUMB mode depending on the global
# setting.
CSRC = $(STARTUPSRC) \
       $(KERNSRC) \
       $(PORTSRC) \
       $(OSALSRC) \
       $(HALSRC) \
       $(PLATFORMSRC) \
       $(BOARDSRC)

# C++ sources that can be compiled in ARM or THUMB mode depending on the global
# setting.
CPPSRC =

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
ASMSRC = $(STARTUPASM) $(PORTASM) $(OSALASM)

INCDIR = $(STARTUPINC) $(KERNINC) $(PORTINC) $(OSALINC) \
         $(HALINC) $(PLATFORMINC) \
         $(CHIBIOS)/os/various src

include app_src.mk

include $(CHIBIOS)/os/various/cpp_wrappers/chcpp.mk
CPPSRC += $(CHIBIOS)/os/various/cpp_wrappers/ch.cpp
INCDIR += $(CHCPPINC)

UAVCAN = ../lib/uavcan

include $(UAVCAN)/libuavcan/include.mk
include $(UAVCAN)/libuavcan_drivers/stm32/driver/include.mk

CPPSRC += $(LIBUAVCAN_SRC) $(LIBUAVCAN_STM32_SRC)
INCDIR += $(LIBUAVCAN_INC) $(LIBUAVCAN_STM32_INC) dsdlc_generated
DDEFS += -DUAVCAN_TOSTRING=0 \
		 -DUAVCAN_STM32_NUM_IFACES=1 \
		 -DUAVCAN_STM32_TIMER_NUMBER=2 \
		 -DUAVCAN_STM32_CHIBIOS=1 \
		 -DUAVCAN_TINY=1

# If we use bootloader we don't have enough room to turn on assertions
ifeq ($(USE_BOOTLOADER), yes)
DDEFS += -DUAVCAN_NO_ASSERTIONS=1
endif


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
LD   = $(TRGT)gcc
#LD   = $(TRGT)g++
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
CWARN = -Wall -Wextra -Wundef -Wstrict-prototypes

# Define C++ warning options here
CPPWARN = -Wall -Wextra -Wundef

#
# Compiler settings
##############################################################################

##############################################################################
# Start of user section
#

# List all user C define here, like -D_DEBUG=1
UDEFS =

# Define ASM defines here
UADEFS =

# List all user directories here
UINCDIR =

# List the user directory to look for the libraries here
ULIBDIR =

# List all user libraries here
ULIBS =

#
# End of user defines
##############################################################################

RULESPATH = $(CHIBIOS)/os/common/ports/ARMCMX/compilers/GCC/
include $(RULESPATH)/rules.mk

.PHONY: flash
flash: build/$(PROJECT).elf
	openocd -f openocd.cfg -c "program build/$(PROJECT).elf verify reset" -c "shutdown"

.PHONY: dsdlc
dsdlc:
	@$(COLOR_PRINTF) "Running uavcan dsdl compiler"
	$(LIBUAVCAN_DSDLC) $(UAVCAN_DSDL_DIR)

