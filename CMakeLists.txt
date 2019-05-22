project(cvra)
cmake_minimum_required(VERSION 3.14)

set(CMAKE_C_STANDARD 99)
set(CMAKE_CXX_STANDARD 14)
enable_language(ASM)

# enable all useful warnings
set(WARN_FLAGS "-Wall -Wextra -Wundef \
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
