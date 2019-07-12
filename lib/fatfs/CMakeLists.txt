if (${CMAKE_CROSSCOMPILING})
    add_library(fatfs
        ff.c
    )

    target_include_directories(fatfs PUBLIC include)
    target_link_libraries(fatfs chibios)
endif()
