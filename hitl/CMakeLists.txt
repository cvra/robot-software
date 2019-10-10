add_executable(motor_board_emulator
    motor_board_emulator.cpp
)

target_link_libraries(motor_board_emulator
    absl::flags
    absl::flags_parse
    hitl_logging
    error
    uavcan
    uavcan_linux
)

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

