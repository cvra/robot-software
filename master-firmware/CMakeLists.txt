add_library(master_lib
    src/unix_timestamp.c
    src/can/bus_enumerator.c
    src/math/lie_groups.c
    src/robot_helpers/math_helpers.c
    src/robot_helpers/beacon_helpers.c
    src/base/base_helpers.c
    src/strategy/state.cpp
    src/strategy/score.cpp
    src/msgbus_protobuf.c
)

target_include_directories(master_lib PUBLIC src)

target_link_libraries(master_lib
    aversive
    pid
    msgbus
    master_proto
    nanopb
    timestamp
    goap
)

cvra_add_test(TARGET master_test
    SOURCES
    tests/unix-timestamp.cpp
    tests/bus_enumerator.cpp
    tests/test_math_helpers.cpp
    tests/test_beacon_helpers.cpp
    tests/obstacle_avoidance.cpp
    tests/trajectory_manager_test.cpp
    tests/lie_groups.cpp
    tests/test_base_helpers.cpp
    tests/test_strategy.cpp
    tests/strategy/test_score.cpp
    tests/msgbus_protobuf.cpp
    # TODO: The following tests depend on injecting a fake ch.h which is harder
    # to do using CMake, so they should be refactored not to depend on it.
    # tests/ch.cpp
    # tests/test_strategy_helpers.cpp
    # tests/test_trajectory_helpers.cpp
    # tests/test_map.cpp
    DEPENDENCIES
    master_lib
    msgbus_mocks_synchronization
)

# List of all protobuf files
set(PROTOSRC
    protobuf/ally_position.proto
    protobuf/beacons.proto
    protobuf/encoders.proto
    protobuf/manipulator.proto
    protobuf/position.proto
    protobuf/protocol.proto
    protobuf/sensors.proto
    protobuf/strategy.proto
    protobuf/Timestamp.proto
)

# Generater .pb.c filenames from .proto filenames
foreach (F ${PROTOSRC})
    get_filename_component(basename ${F} NAME_WE)
    list(APPEND PROTOCSRC ${CMAKE_BINARY_DIR}/protobuf/protobuf/${basename}.pb.c)
endforeach()

add_custom_target(protoc)

file(MAKE_DIRECTORY ${CMAKE_BINARY_DIR}/protobuf/protobuf/)

# TODO Inputs
add_custom_command(
    COMMAND protoc
        --plugin=protoc-gen-nanopb=../lib/nanopb/nanopb/generator/protoc-gen-nanopb
        --python_out=${CMAKE_BINARY_DIR}/protobuf
		-I ../lib/nanopb/nanopb/generator/proto
		-I protobuf/
        --nanopb_out=${CMAKE_BINARY_DIR}/protobuf/protobuf
        ${PROTOSRC}
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
    OUTPUT ${PROTOCSRC}
    COMMENT "Running protobuf compiler..."
)

add_library(master_proto
    ${PROTOCSRC}
)

target_link_libraries(master_proto nanopb)
target_include_directories(master_proto PUBLIC ${CMAKE_BINARY_DIR}/protobuf)

# Generate code for the Msgpack config files
file(MAKE_DIRECTORY ${CMAKE_BINARY_DIR}/config)
add_custom_command(
    COMMAND tools/config/config_to_msgpack.py
            --name=msgpack_config_order
            config_order.yaml
            ${CMAKE_BINARY_DIR}/config/config_order.c
    OUTPUT ${CMAKE_BINARY_DIR}/config/config_order.c
    WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}
    MAIN_DEPENDENCY ../config_order.yaml
    COMMENT "Generating Order config"
)
add_custom_command(
    COMMAND tools/config/config_to_msgpack.py
            --name=msgpack_config_chaos
            config_chaos.yaml
            ${CMAKE_BINARY_DIR}/config/config_chaos.c
    OUTPUT ${CMAKE_BINARY_DIR}/config/config_chaos.c
    WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}
    MAIN_DEPENDENCY ../config_chaos.yaml
    COMMENT "Generating Chaos config"
)
add_custom_command(
    COMMAND tools/config/config_to_c.py
            config_chaos.yaml
            ${CMAKE_BINARY_DIR}/config/config_private.h
    OUTPUT ${CMAKE_BINARY_DIR}/config/config_private.h
    WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}
    MAIN_DEPENDENCY ../config_chaos.yaml
    COMMENT "Generating config structure"
)
add_library(config_data
    ${CMAKE_BINARY_DIR}/config/config_order.c
    ${CMAKE_BINARY_DIR}/config/config_chaos.c
)
target_include_directories(config_data PUBLIC ${CMAKE_BINARY_DIR}/config)

if (${CMAKE_CROSSCOMPILING} AND "${CVRA_BOARD_NAME}" STREQUAL "master")
    set(CHIBIOS_HALCONF_FILE src/halconf.h)
    set(CHIBIOS_CHCONF_FILE src/chconf.h)
    set(CHIBIOS_ROOT ../lib/ChibiOS)
    find_package(ChibiOS COMPONENTS
        rt
        hal
        streams
        shell
        cppwrappers
        evtimer
    REQUIRED)

    add_library(chibios ${ChibiOS_SOURCES})
    target_include_directories(chibios PUBLIC
        ${ChibiOS_INCLUDE_DIRS}
        src # for chconf
        src/board # for board.h
    )
    target_compile_definitions(chibios PRIVATE -DSHELL_CONFIG_FILE)
    target_link_libraries(chibios fatfs_conf)

    add_executable(master-firmware.elf
        #src/adafruit_display_driver/board_hx8357.c
        #src/adafruit_display_driver/gdisp_lld_hx8357.c
        src/ally_position_service.c
        src/base/base_controller.c
        src/base/cs_port.c
        src/base/encoder.c
        src/base/map_server.cpp
        src/base/rs_port.c
        src/board/board.c
        src/can/beacon_signal_handler.cpp
        src/can/can_io_driver.cpp
        src/can/can_uwb_ip_netif.cpp
        src/can/electron_starter.cpp
        src/can/emergency_stop_handler.cpp
        src/can/motor_driver.c
        src/can/motor_manager.c
        src/can/motor_driver_uavcan.cpp
        src/can/sensor_handler.cpp
        src/can/uavcan_node.cpp
        src/can/uwb_position_handler.cpp
        src/can/sensor_handler.cpp
        src/can/uavcan_node.cpp
        src/can/uwb_position_handler.cpp
        src/can/motor_feedback_streams_handler.cpp
        src/commands.cpp
        src/config.c
        src/control_panel.c
        src/debug/blocking_uart.c
        src/debug/log.c
        src/debug/panic_log.c
        src/filesystem.c
        #src/gui.cpp
        #src/gui/Menu.cpp
        #src/gui/MenuPage.cpp
        src/http/server.c
        src/main.c
        src/parameter_port.c
        src/pca9685_pwm.c
        src/robot_helpers/motor_helpers.c
        src/strategy.cpp
        src/strategy/score_counter.cpp
        src/trace/trace_points.c
        src/trace/trace_port.c
        src/udp_topic_broadcaster.c
        src/uid.c
        src/usbconf.c

        # TODO: Should this be in its own library ?
        ../lib/ChibiOS/os/various/fatfs_bindings/fatfs_diskio.c
        ../lib/ChibiOS/os/various/fatfs_bindings/fatfs_syscall.c

        # TODO: Those files should be unit tested but depend on injecting fake
        # header files, which is harder to do now with cmake
        src/base/map.c
        src/robot_helpers/trajectory_helpers.c
        src/robot_helpers/strategy_helpers.c
    )

    target_link_libraries(master-firmware.elf
        chibios
        chibios-syscalls
        parameter
        nanopb
        msgbus
        msgbus_chibios
        error
        arm_cortex_tools
        timestamp
        timestamp_stm32
        lwip
        quadramp
        aversive
        pid
        trace
        master_proto
        uavcan
        uavcan_stm32
        fatfs
        goap
        crc
        master_lib
        config_data
    )

    add_library(lwipopt INTERFACE)
    target_include_directories(lwipopt INTERFACE src/)
    target_link_libraries(lwipopt INTERFACE chibios)

    add_library(lwip_bindings
        src/lwip_bindings/lwipthread.c
        src/lwip_bindings/arch/sys_arch.c
    )

    target_include_directories(lwip_bindings PUBLIC src/lwip_bindings)
    target_link_libraries(lwip_bindings PUBLIC
        chibios
        crc
        error
        lwip
        parameter
    )

    target_include_directories(master-firmware.elf PUBLIC
        src
        # TODO delete the includes below and rather prefix them in include
        # statements
        src/can
        src/debug
        src/lwip_bindings
    )

    stm32_set_linker_script(master-firmware.elf linker/STM32F429xI.ld)
    stm32_dfu_upload(master-firmware.elf)
endif()
