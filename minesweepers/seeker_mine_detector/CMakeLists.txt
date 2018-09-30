cmake_minimum_required(VERSION 2.8.3)
project(seeker_mine_detector)

find_package(catkin
    REQUIRED
        seeker_msgs
)

catkin_package()

install(PROGRAMS
    nodes/emi_mine_detector.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
