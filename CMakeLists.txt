cmake_minimum_required(VERSION 2.6)
project(msgbus)
set(CMAKE_BUILD_TYPE Debug)

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x -Wall")
SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall")

add_executable(
    tests
    ./messagebus.c
    ./tests/atomicity.cpp
    ./tests/signaling.cpp
    ./tests/mocks/synchronization.cpp
    ./tests/msgbus.cpp
    ./tests/main.cpp
    )

target_link_libraries(
    tests
    CppUTest
    CppUTestExt
    )

add_executable(
    demo
    ./messagebus.c
    ./examples/posix/port.c
    ./examples/posix/demo.c
    )

target_link_libraries(
    demo
    pthread
    )

# Run unit tests
add_custom_target(check ./tests -c DEPENDS tests)
