# Add cmake/ to the CMake path
get_filename_component(stm32_cmake_dir ${CMAKE_CURRENT_LIST_FILE} DIRECTORY)
set(CMAKE_MODULE_PATH ${stm32_cmake_dir} ${CMAKE_MODULE_PATH})

set(STM32_CHIP "STM32F302K8" CACHE STRING "STM32 chip to build for")
set(STM32_FAMILY "F3" CACHE INTERNAL "stm32 family")
set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_PROCESSOR arm)
set(CVRA_BOARD_NAME can-io)

set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)
set(CMAKE_C_COMPILER "arm-none-eabi-gcc")
set(CMAKE_CXX_COMPILER "arm-none-eabi-g++")
set(CMAKE_ASM_COMPILER "arm-none-eabi-gcc")
set(CMAKE_OBJCOPY "arm-none-eabi-objcopy" CACHE INTERNAL "objcopy tool")
set(CMAKE_OBJDUMP "arm-none-eabi-objdump" CACHE INTERNAL "objdump tool")
set(CMAKE_SIZE "arm-none-eabi-size" CACHE INTERNAL "size tool")
set(CMAKE_DEBUGER "arm-none-eabi-gdb" CACHE INTERNAL "debuger")
set(CMAKE_CPPFILT "arm-none-eabi-c++filt" CACHE INTERNAL "C++filt")
set(CMAKE_AR "arm-none-eabi-ar" CACHE INTERNAL "ar")
set(CMAKE_RANLIB "arm-none-eabi-ranlib" CACHE INTERNAL "ranlib")

set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)

# Set flags common to C and C++ such as hardware platform, ABI and so on
set(HARDWARE_FLAGS 
    "-mthumb \
    -mno-thumb-interwork \
    -mcpu=cortex-m4 \
    -mabi=aapcs \
    -falign-functions=16"
    CACHE INTERNAL "hardware related flags")

set(COMMON_FLAGS "${HARDWARE_FLAGS} \
    -fno-builtin \
    -ffunction-sections -fdata-sections \
    -fomit-frame-pointer \
    -fno-unroll-loops \
    -ffast-math \
    -ftree-vectorize \
    -Os \
    " CACHE INTERNAL "common C and C++ flags")

set(CMAKE_C_FLAGS "${COMMON_FLAGS}" CACHE INTERNAL "c compiler flags")

# Set C++ specific options, mostly disabling expensive features
set(CMAKE_CXX_FLAGS "${COMMON_FLAGS} \
    -fno-exceptions -fno-unwind-tables \
    -fno-threadsafe-statics -fno-rtti \
    -fno-use-cxa-atexit \
    "
    CACHE INTERNAL "cxx compiler flags")

set(CMAKE_ASM_FLAGS "${HARDWARE_FLAGS} -x assembler-with-cpp"
    CACHE INTERNAL "asm compiler flags")

set(CMAKE_EXE_LINKER_FLAGS "-Wl,--gc-sections ${HARDWARE_FLAGS}"
    CACHE INTERNAL "executable linker flags")

set(CMAKE_MODULE_LINKER_FLAGS "${HARDWARE_FLAGS}" CACHE INTERNAL "module linker flags")

set(CMAKE_SHARED_LINKER_FLAGS "${HARDWARE_FLAGS}" CACHE INTERNAL "shared linker flags")

add_compile_definitions("CORTEX_USE_FPU=FALSE")
add_compile_definitions("THUMB")
add_compile_definitions("THUMB_PRESENT")
add_compile_definitions("THUMB_NO_INTERWORKING")
add_compile_definitions("SHELL_CMD_TEST_ENABLED=FALSE")
add_compile_definitions("UAVCAN_STM32_TIMER_NUMBER=6")
add_compile_definitions("UAVCAN_TINY=1")
add_compile_definitions("CVRA_NO_DYNAMIC_ALLOCATION=1")

include(stm32_common)
