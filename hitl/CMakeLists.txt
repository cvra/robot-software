
add_executable(motor_board_emulator
    motor_board_emulator.cpp
)

target_link_libraries(motor_board_emulator
    uavcan
)

