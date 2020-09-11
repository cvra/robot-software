find_package(Threads)
add_library(master_lib
    src/unix_timestamp.c
    src/can/bus_enumerator.c
    src/math/lie_groups.c
    src/robot_helpers/math_helpers.c
    src/robot_helpers/beacon_helpers.cpp
    src/strategy/state.cpp
    src/strategy/score.cpp
    src/strategy/actions_goap.cpp
    src/strategy/goals.cpp
    src/msgbus_protobuf.c
)

target_include_directories(master_lib PUBLIC src)

target_link_libraries(master_lib
    aversive
    pid
    msgbus
    msgbus_posix
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
    tests/trajectory_manager_test.cpp
    tests/lie_groups.cpp
    tests/test_strategy.cpp
    tests/strategy/test_score.cpp
    tests/strategy/test_actions.cpp
    tests/strategy/test_goals.cpp
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

cvra_add_test(TARGET uavcan_tests
    SOURCES
    tests/uavcan_to_messagebus_test.cpp
    DEPENDENCIES
    master_lib
    uavcan
    uavcan_linux
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
    ../config_simulation.yaml
    ${CMAKE_CURRENT_BINARY_DIR}/config_private/config_private.h
    OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/config_private/config_private.h
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
    COMMENT "Generating config structure"
    DEPENDS ../config_order.yaml ../config_chaos.yaml ../config_simulation.yaml
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
    COMMAND tools/config/config_to_msgpack.py
            --name=msgpack_config_simulation
            config_simulation.yaml
            ${CMAKE_BINARY_DIR}/config/config_simulation.c
    OUTPUT ${CMAKE_BINARY_DIR}/config/config_simulation.c
    WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}
    MAIN_DEPENDENCY ../config_simulation.yaml
    COMMENT "Generating simulation config"
)
add_library(config_data
    ${CMAKE_BINARY_DIR}/config/config_order.c
    ${CMAKE_BINARY_DIR}/config/config_chaos.c
    ${CMAKE_BINARY_DIR}/config/config_simulation.c
)
target_include_directories(config_data PUBLIC ${CMAKE_BINARY_DIR}/config)

add_library(gpioinput
    src/gpio_input.cpp
)

target_link_libraries(gpioinput PUBLIC gpiod error absl::base)

add_executable(master-firmware
    src/main.cpp
    src/debug/log.c
    src/can/uavcan_node.cpp
    src/can/motor_manager.c
    src/can/motor_driver.c
    src/can/motor_driver_uavcan.cpp
    src/can/wheel_encoders_handler.cpp
    src/can/emergency_stop_handler.cpp
    src/can/motor_feedback_streams_handler.cpp
    src/can/can_io_driver.cpp
    src/control_panel.cpp
    src/config.c
    src/parameter_port.cpp
    src/base/base_controller.cpp
    src/base/rs_port.c
    src/base/cs_port.c
    src/gui.cpp
    src/gui/Menu.cpp
    src/gui/MenuPage.cpp
    src/strategy/actions_impl.cpp
    src/strategy.cpp
    src/robot_helpers/trajectory_helpers.cpp
)

target_link_libraries(master-firmware PUBLIC
    absl::strings
    absl::flags
    absl::flags_parse
    absl::synchronization
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
    gpioinput
)

target_include_directories(master-firmware PUBLIC src
)

add_custom_target(master-firmware.ipk
    COMMAND ${CMAKE_CURRENT_SOURCE_DIR}/../tools/build_opkg_package.py
    --name master-firmware
    --output master-firmware.ipk
    master-firmware:/bin/cvra-master
    ${CMAKE_CURRENT_SOURCE_DIR}/packaging/cvra-master.service:/usr/lib/systemd/system/cvra-master.service
    --postinst ${CMAKE_CURRENT_SOURCE_DIR}/packaging/postinst
    --prerm ${CMAKE_CURRENT_SOURCE_DIR}/packaging/prerm
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
