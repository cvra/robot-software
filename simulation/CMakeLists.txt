cmake_minimum_required(VERSION 2.6)

project(cvra-simulation)
set(CMAKE_BUILD_TYPE Debug)

include_directories(../src/aversive/)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DTESTS -std=c++0x -Wall -Wextra")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -DTESTS -std=gnu99 -Wall -Wextra -Wno-char-subscripts")

add_executable(
    simulation
    ../src/aversive/math/geometry/circles.c
    ../src/aversive/math/geometry/lines.c
    ../src/aversive/math/geometry/polygon.c
    ../src/aversive/math/geometry/vect_base.c
    ../src/aversive/obstacle_avoidance/obstacle_avoidance.c
    visualizer.cpp
    main.cpp
    )

target_link_libraries(
    simulation
    GL
    GLU
    glut
    m
    X11
    Xext
    Xmu
    Xi
    )

# Run unit tests
add_custom_target(run ./simulation -c DEPENDS simulation)
