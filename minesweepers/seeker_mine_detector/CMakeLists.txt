cmake_minimum_required(VERSION 2.8.3)
project(seeker_mine_detector)

add_compile_options(-std=c++17)

find_package(catkin REQUIRED
    COMPONENTS
        pcl_conversions
        pcl_ros
        roscpp
        seeker_msgs
        sensor_msgs
)

catkin_package()

include_directories(
    ${catkin_INCLUDE_DIRS}
)

add_executable(segmentation_node
    src/segmentation_node.cpp
)
add_dependencies(segmentation_node
    ${${PROJECT_NAME}_EXPORTED_TARGETS}
    ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(segmentation_node
    ${catkin_LIBRARIES}
)

install(TARGETS segmentation_node
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
