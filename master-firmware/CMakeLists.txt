find_package(Threads)
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
    msgbus
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
    DEPENDS ${PROTOSRC}
)

add_library(master_proto
    ${PROTOCSRC}
)

file(MAKE_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/config_private/)
add_custom_command(
    COMMAND ../tools/config/config_to_c.py
    ../config_order.yaml
    ../config_chaos.yaml
    ${CMAKE_CURRENT_BINARY_DIR}/config_private/config_private.h
    OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/config_private/config_private.h
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
    COMMENT "Generating config structure"
    DEPENDS ../config_order.yaml ../config_chaos.yaml
)

add_custom_target(master_config_header ALL DEPENDS
    ${CMAKE_CURRENT_BINARY_DIR}/config_private/config_private.h
)

add_library(master_config_structure INTERFACE)

target_include_directories(master_config_structure INTERFACE
    ${CMAKE_CURRENT_BINARY_DIR}/config_private/
)

add_dependencies(master_config_structure master_config_header)


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

add_executable(master-firmware
    src/main.cpp
    src/debug/log.c
    src/can/uavcan_node.cpp
    src/can/motor_manager.c
    src/can/motor_driver.c
    src/can/motor_driver_uavcan.cpp
    src/control_panel.c
    src/config.c
    src/parameter_port.cpp
    src/base/base_controller.cpp
    src/base/rs_port.c
    src/base/cs_port.c
    src/gui.cpp
    src/gui/Menu.cpp
    src/gui/MenuPage.cpp
)

target_link_libraries(master-firmware PUBLIC
    absl::strings
    absl::flags
    absl::flags_parse
    error
    uavcan
    uavcan_linux
    msgbus
    msgbus_posix
    Threads::Threads
    master_lib
    parameter
    ugfx
    config_data
    master_config_structure
)

target_include_directories(master-firmware PUBLIC src
)

add_custom_target(master-firmware.ipk
    COMMAND ${CMAKE_CURRENT_SOURCE_DIR}/../tools/build_opkg_package.py
    --name master-firmware
    --output master-firmware.ipk
    master-firmware:/usr/local/bin/cvra-master
    DEPENDS master-firmware
    COMMENT "Packaging master-firmware.ipk"
)

add_library(gfxconf INTERFACE)
target_include_directories(gfxconf INTERFACE
    ../ugfx_demo
)

if (APPLE) # MacOS
    add_definitions(-DGFX_USE_OS_OSX=GFXON)
elseif (UNIX) # Linux
    add_definitions(-DGFX_USE_OS_LINUX=GFXON)
else()
    message(FATAL_ERROR "Supported platforms: Linux and MacOS")
endif()

target_include_directories(ugfx PUBLIC ../lib/ugfx/drivers/multiple/SDL)


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
        src/base/base_controller.cpp
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
        src/gui.cpp
        src/gui/Menu.cpp
        src/gui/MenuPage.cpp
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

        # TODO: This cannot go in a lib because of linking order issues
        ../lib/chibios-syscalls/newlib_syscalls.c

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
        ugfx
        adafruit_display_driver

        # touchscreen driver
        stmpe610_driver
    )

    add_library(lwipopt INTERFACE)
    target_include_directories(lwipopt INTERFACE src/)
    target_link_libraries(lwipopt INTERFACE chibios)

    add_library(gfxconf INTERFACE)
    target_include_directories(gfxconf INTERFACE
        src/
        src/adafruit_display_driver/
    )

    add_library(adafruit_display_driver
        src/adafruit_display_driver/gdisp_lld_hx8357.c
        src/adafruit_display_driver/board_hx8357.c
    )

    target_link_libraries(adafruit_display_driver ugfx)

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

    target_include_directories(master-firmware.elf PUBLIC src)

    stm32_set_linker_script(master-firmware.elf linker/STM32F429xI.ld)
endif()


