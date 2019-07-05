project(cvra)
cmake_minimum_required(VERSION 3.12)

set(CMAKE_C_STANDARD 99)
set(CMAKE_CXX_STANDARD 14)
enable_language(ASM)

# Allow us to have colored text
option (FORCE_COLORED_OUTPUT "Always produce ANSI-colored output (GNU/Clang only)." TRUE)
if (${FORCE_COLORED_OUTPUT})
    if ("${CMAKE_CXX_COMPILER_ID}" STREQUAL "GNU")
       add_compile_options (-fdiagnostics-color=always)
    elseif ("${CMAKE_CXX_COMPILER_ID}" STREQUAL "Clang")
       add_compile_options (-fcolor-diagnostics)
    endif ()
endif ()

# enable all useful warnings
set(WARN_FLAGS "-Wall -Wextra -Wundef \
    -Wno-error=unknown-warning \
    -Wduplicated-branches -Wduplicated-cond \
    -Wlogical-op \
    -Wnull-dereference \
    -Wshadow \
    ")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${WARN_FLAGS} -Wstrict-prototypes")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${WARN_FLAGS}")

set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

include(cvra)

enable_testing()

add_subdirectory(lib)
add_subdirectory(uwb-beacon-firmware)
