cmake_minimum_required(VERSION 2.8.3)
project(seeker_msgs)

find_package(catkin
    REQUIRED
        message_generation
        std_msgs
        geometry_msgs
)

add_message_files(
    FILES
        MineInfo.msg
        MineMap.msg
)

generate_messages(
    DEPENDENCIES
        std_msgs
        geometry_msgs
)

catkin_package()
