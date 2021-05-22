add_executable(uavcan_controller
    EXCLUDE_FROM_ALL
    uavcan_controller.cpp
)


target_link_libraries(uavcan_controller PUBLIC uavcan uavcan_linux hitl_lib)

# Integrate with Webots libs
target_include_directories(uavcan_controller PUBLIC /usr/local/webots/include/controller/cpp)
link_directories(/usr/local/webots/lib/controller)
target_link_libraries(uavcan_controller PUBLIC Controller CppController)
