cmake_minimum_required(VERSION 3.10)
project(cvra_strategy_simulator)

set(CMAKE_CXX_STANDARD 14)

set(SOURCES
    main.cpp
    simulation.cpp
    msgbus_port.cpp
    parameter_port.cpp
    ch.cpp
    ../master-firmware/src/strategy_impl/actions.cpp
    ../master-firmware/src/strategy/state.cpp
    ../master-firmware/src/strategy/score.cpp
    ../master-firmware/src/base/map.c
    ../master-firmware/src/robot_helpers/math_helpers.c
    ../master-firmware/build/protobuf/protocol.pb.c
    ../master-firmware/build/protobuf/position.pb.c
    ../master-firmware/build/protobuf/strategy.pb.c
    ../lib/quadramp/quadramp.c
    ../lib/msgbus/messagebus.c
    ../lib/aversive/blocking_detection_manager/blocking_detection_manager.c
    ../lib/aversive/control_system_manager/control_system_manager.c
    ../lib/aversive/math/geometry/circles.c
    ../lib/aversive/math/geometry/discrete_circles.c
    ../lib/aversive/math/geometry/lines.c
    ../lib/aversive/math/geometry/polygon.c
    ../lib/aversive/math/geometry/vect_base.c
    ../lib/aversive/math/vect2/vect2.c
    ../lib/aversive/obstacle_avoidance/obstacle_avoidance.c
    ../lib/aversive/position_manager/position_manager.c
    ../lib/aversive/robot_system/robot_system.c
    ../lib/aversive/robot_system/angle_distance.c
    ../lib/aversive/trajectory_manager/trajectory_manager.c
    ../lib/aversive/trajectory_manager/trajectory_manager_core.c
    ../lib/aversive/trajectory_manager/trajectory_manager_utils.c
    ../lib/nanopb/nanopb/pb_common.c
    ../lib/nanopb/nanopb/pb_decode.c
    ../lib/nanopb/nanopb/pb_encode.c
    ../lib/error/error.c
    ../lib/parameter/parameter.c
)

add_executable(simulator ${SOURCES} )
target_include_directories(simulator
    PUBLIC
        .
        ../master-firmware/src/
        ../master-firmware/build/
        ../lib/
        ../lib/nanopb/nanopb/
)
target_link_libraries(simulator ${SDL2_LIBRARIES})

if (UNIX AND NOT APPLE)
    target_link_libraries(simulator pthread)
endif()
