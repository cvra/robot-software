if (${CMAKE_CROSSCOMPILING})
    add_library(fatfs
        ff.c
    )

    target_include_directories(fatfs PUBLIC include)
    target_link_libraries(fatfs chibios)

    # This library contains only ffconf.h, which is required by ChibiOS
    add_library(fatfs_conf INTERFACE)
    target_include_directories(fatfs_conf INTERFACE include/fatfs)
endif()
