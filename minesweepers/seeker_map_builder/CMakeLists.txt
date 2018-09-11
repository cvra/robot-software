cmake_minimum_required(VERSION 2.8.3)
project(seeker_map_builder)

add_compile_options(-std=c++11)

find_package(catkin
    REQUIRED
        roscpp
        seeker_msgs
)

catkin_package()

include_directories(
    ${catkin_INCLUDE_DIRS}
)

add_executable(map_builder src/map_builder.cpp)
target_link_libraries(map_builder ${catkin_LIBRARIES})

install(
    TARGETS
        map_builder
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
