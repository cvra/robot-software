find_package(OpenGL REQUIRED)
find_package(GLUT REQUIRED)
find_package(PNG REQUIRED)

add_library(physics PhysicsRobot.cpp PhysicsCup.cpp OpponentRobot.cpp)
target_link_libraries(physics PUBLIC box2d)

cvra_add_test(TARGET box2d_test
    SOURCES tests/box2d.cpp
    DEPENDENCIES physics
)

add_executable(motor_board_emulator
    main.cpp
    motor_board_emulator.cpp
    wheel_encoders_emulator.cpp
    sensor_board_emulator.cpp
    actuator_board_emulator.cpp
    viewer.cpp
    png_loader.cpp
    actuator_board_emulator.cpp
    ProximityBeaconEmulator.cpp
)

target_link_libraries(motor_board_emulator
    absl::flags
    absl::flags_parse
    hitl_logging
    error
    uavcan
    uavcan_linux
    box2d
    physics
    ${OPENGL_gl_LIBRARY}
    ${GLUT_LIBRARIES}
    PNG::PNG
)

add_library(hitl_lib
    motor_board_emulator.cpp
    wheel_encoders_emulator.cpp
    sensor_board_emulator.cpp
    actuator_board_emulator.cpp
)

target_link_libraries(hitl_lib
    hitl_logging
    error
    uavcan
    uavcan_linux
)

target_include_directories(hitl_lib PUBLIC .)



configure_file(table.png table.png COPYONLY)

add_library(hitl_logging
    logging.cpp
    logging.h
)

target_link_libraries(hitl_logging
    absl::flags
    error
)

add_executable(voltage_injector
    voltage_injector.cpp
)

target_link_libraries(voltage_injector
    absl::flags
    absl::flags_parse
    hitl_logging
    error
    uavcan
    uavcan_linux
)

