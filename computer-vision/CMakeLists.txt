cmake_minimum_required(VERSION 3.0 FATAL_ERROR)
project(robot_tracking VERSION 0.1.0 LANGUAGES C CXX)

find_package(Protobuf REQUIRED)
find_package(OpenCV REQUIRED)

message(STATUS "OpenCV library status:")
message(STATUS "    config: ${OpenCV_DIR}")
message(STATUS "    version: ${OpenCV_VERSION}")
message(STATUS "    libraries: ${OpenCV_LIBS}")
message(STATUS "    include path: ${OpenCV_INCLUDE_DIRS}")

set(CMAKE_CXX_STANDARD 11)
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall")

add_executable(example example.cpp)
target_link_libraries(example LINK_PRIVATE ${OpenCV_LIBS})

# List of all protobuf files
set(PROTODIR ../master-firmware/protobuf)
set(PROTOSRC
    ${PROTODIR}/protocol.proto
    ${PROTODIR}/computer_vision.proto
    ../lib/nanopb/nanopb/generator/proto/nanopb.proto
)

# Generater .pb.cc filenames from .proto filenames
foreach (F ${PROTOSRC})
    get_filename_component(basename ${F} NAME_WE)
    list(APPEND PROTOCSRC ${CMAKE_BINARY_DIR}/protobuf/protobuf/${basename}.pb.cc)
endforeach()

add_custom_target(protoc)

file(MAKE_DIRECTORY ${CMAKE_BINARY_DIR}/protobuf/protobuf/)

add_custom_command(
    COMMAND protoc
        -I ${PROTODIR}
        -I ../lib/nanopb/nanopb/generator/proto
        --cpp_out=${CMAKE_BINARY_DIR}/protobuf/protobuf
        ${PROTOSRC}
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
    OUTPUT ${PROTOCSRC}
    COMMENT "Running protobuf compiler..."
    DEPENDS ${PROTOSRC}
)

add_library(master_proto
    ${PROTOCSRC}
)
target_include_directories(master_proto PUBLIC ${Protobuf_INCLUDE_DIRS} ${CMAKE_BINARY_DIR}/protobuf)

add_library(error ../lib/error/error.c)
target_include_directories(error PUBLIC ../lib/)

add_executable(robot_tracking
    robot_tracking.cpp
    logging.cpp
    udp_protocol.cpp
)
target_link_libraries(robot_tracking
    ${OpenCV_LIBS}
    master_proto
    ${Protobuf_LIBRARIES}
    error
)
