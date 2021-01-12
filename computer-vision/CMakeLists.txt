find_package(OpenCV REQUIRED)
find_package(Protobuf REQUIRED)

message(STATUS "OpenCV library status:")
message(STATUS "    config: ${OpenCV_DIR}")
message(STATUS "    version: ${OpenCV_VERSION}")
message(STATUS "    libraries: ${OpenCV_LIBS}")
message(STATUS "    include path: ${OpenCV_INCLUDE_DIRS}")

set(CMAKE_CXX_STANDARD 11)
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall")

protobuf_generate_cpp(PROTO_SRCS PROTO_HDRS
    ../master-firmware/protobuf/computer_vision.proto
    ../master-firmware/protobuf/protocol.proto
    ../lib/nanopb/nanopb/generator/proto/nanopb.proto
    )

add_executable(robot_tracking
    robot_tracking.cpp
    logging.cpp
    udp_protocol.cpp
    ${PROTO_SRCS}
    ${PROTO_HDRS}
)



target_link_libraries(robot_tracking
    opencv_highgui
    opencv_aruco
    ${Protobuf_LIBRARY}
    error
)

target_include_directories(robot_tracking PUBLIC ${CMAKE_CURRENT_BINARY_DIR})
