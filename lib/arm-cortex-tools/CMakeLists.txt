if (${CMAKE_CROSSCOMPILING})
    add_library(arm_cortex_tools
        mpu.c
        fault.c
        fault_v7m.s
    )

    target_include_directories(arm_cortex_tools
        PUBLIC include
    )

    target_link_libraries(arm_cortex_tools chibios)
endif()
