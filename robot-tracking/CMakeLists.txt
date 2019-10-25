cmake_minimum_required(VERSION 3.0 FATAL_ERROR)
project(robot_tracking VERSION 0.1.0 LANGUAGES CXX)

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


add_executable(robot_tracking robot_tracking.cpp)
target_link_libraries(robot_tracking LINK_PRIVATE ${OpenCV_LIBS})
