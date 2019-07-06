add_library(state_estimation
    src/state_estimation.cpp
)

target_include_directories(state_estimation PUBLIC src)

target_link_libraries(state_estimation
    Eigen
)

cvra_add_test(TARGET ekf_test SOURCES
    tests/ekf.cpp
    tests/state_estimation.cpp
    DEPENDENCIES
    Eigen
    state_estimation
)

add_library(lru_cache
    src/lru_cache.c
)

target_include_directories(lru_cache PUBLIC src)

cvra_add_test(TARGET lru_cache_test SOURCES
    tests/lru_cache.cpp
    DEPENDENCIES
    lru_cache
)

add_library(mpu9250
    src/mpu9250.c
)

target_include_directories(mpu9250 PUBLIC src)

cvra_add_test(TARGET mpu9250_test SOURCES
    tests/mpu9250.cpp
    DEPENDENCIES
    mpu9250
)

add_library(madgwick
    src/MadgwickAHRS.c
)

target_include_directories(madgwick PUBLIC src)

cvra_add_test(TARGET madgwick_test SOURCES
    tests/madgwick.cpp
    DEPENDENCIES
    madgwick
)

add_library(decawave
    src/decadriver/deca_device.c
    src/decadriver/deca_params_init.c
)

target_include_directories(decawave PUBLIC src)

cvra_add_test(TARGET decawave_api_test SOURCES
    tests/decawave.cpp
    DEPENDENCIES
    decawave
)

add_library(uwb_protocol
    src/uwb_protocol.c
)

target_include_directories(uwb_protocol PUBLIC src)

cvra_add_test(TARGET uwb_protocol_test SOURCES
    tests/uwb_protocol.cpp
    DEPENDENCIES
    uwb_protocol
)

if (NOT ${CMAKE_CROSSCOMPILING})
    add_library(fake_hal tests/hal_mocks.cpp)
    target_include_directories(fake_hal PUBLIC tests)
    target_link_libraries(mpu9250 fake_hal)
else()
    target_link_libraries(mpu9250 chibios)
endif()

if (${CMAKE_CROSSCOMPILING} AND "${CVRA_BOARD_NAME}" STREQUAL "uwb-beacon")
    set(CHIBIOS_HALCONF_FILE src/halconf.h)
    set(CHIBIOS_CHCONF_FILE src/chconf.h)


    set(CHIBIOS_ROOT ../lib/ChibiOS)
    find_package(ChibiOS COMPONENTS rt hal streams shell cppwrappers REQUIRED)


    add_library(chibios ${ChibiOS_SOURCES})
    target_include_directories(chibios PUBLIC
        ${ChibiOS_INCLUDE_DIRS}
        src # for chconf
    )

    target_compile_definitions(chibios PRIVATE -DSHELL_CONFIG_FILE)

    add_executable(uwb-beacon-firmware.elf
        src/ahrs_thread.c
        src/anchor_position_cache.c
        src/board.c
        src/cmd.c
        src/decawave_interface.c
        src/exti.c
        src/imu_thread.c
        src/main.c
        src/parameter_port.c
        src/ranging_thread.c
        src/state_estimation_thread.cpp
        src/trace_port.c
        src/uavcan/data_packet_handler.cpp
        src/uavcan/parameter_server.cpp
        src/uavcan/position_handler.cpp
        src/uavcan/restart_server.cpp
        src/uavcan/topics_publisher.cpp
        src/uavcan/uavcan_node.cpp
        src/usbconf.c
        src/usbconf.c
        ../lib/chibios-syscalls/newlib_syscalls.c
    )

    target_link_libraries(uwb-beacon-firmware.elf
        chibios
        parameter_flash_storage
        msgbus
        msgbus_chibios
        lru_cache
        trace
        Eigen
        state_estimation
        madgwick
        mpu9250
        decawave
        parameter_flash_f4
        uwb_protocol
        chibios-syscalls
        uavcan
        uavcan_stm32
    )

    target_include_directories(uwb-beacon-firmware.elf PUBLIC src)

    stm32_set_linker_script(uwb-beacon-firmware.elf STM32F405xG.ld)

    stm32_dfu_upload(uwb-beacon-firmware.elf)
endif()
