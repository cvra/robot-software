project(cvra)
cmake_minimum_required(VERSION 3.10)


set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)
include(cvra)

include(AddCXXCompilerFlag)
include(AddCCompilerFlag)

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
add_c_cxx_compiler_flag("-Wall")
add_c_cxx_compiler_flag("-Wextra")
add_c_cxx_compiler_flag("-Wundef")
add_c_cxx_compiler_flag("-Wduplicated-cond")
add_c_cxx_compiler_flag("-Wlogical-op")
add_c_cxx_compiler_flag("-Wnull-dereference")
add_c_cxx_compiler_flag("-Wshadow")
add_c_compiler_flag("-Wstrict-prototypes")
add_cxx_compiler_flag("-Wthread-safety -Werror=thread-safety")

# The following flags are causing issues with Eigen
add_cxx_compiler_flag("-Wno-duplicated-branches")
add_cxx_compiler_flag("-Wno-deprecated-copy")

enable_testing()

add_subdirectory(lib)
add_subdirectory(uwb-beacon-firmware)
add_subdirectory(master-firmware)
add_subdirectory(computer-vision)

if (NOT ${CMAKE_CROSSCOMPILING})
    add_subdirectory(hitl)
endif()
