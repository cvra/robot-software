if (CMAKE_CROSSCOMPILING)
    add_library(chibios-syscalls
        malloc_lock.c
        newlib_syscalls.c
        cpp_stubs.cpp
        dynamic_alloc_stubs.cpp
        stdio_lock.c
    )

    target_include_directories(chibios-syscalls PUBLIC include)

    target_link_libraries(chibios-syscalls chibios)
endif()
