cmake_minimum_required(VERSION 2.8.3)
project(uavcan_bridge)

find_package(catkin REQUIRED)

catkin_package()

install(PROGRAMS
    nodes/position_publisher.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
