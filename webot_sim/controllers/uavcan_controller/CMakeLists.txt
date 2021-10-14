link_directories(/usr/local/webots/lib/controller)

add_executable(uavcan_controller
    EXCLUDE_FROM_ALL
    uavcan_controller.cpp
)

target_link_libraries(uavcan_controller PUBLIC uavcan uavcan_linux hitl_lib)

# Integrate with Webots libs
target_include_directories(uavcan_controller PUBLIC /usr/local/webots/include/controller/cpp)
target_link_libraries(uavcan_controller PUBLIC Controller CppController)

add_executable(record_animation
    EXCLUDE_FROM_ALL
    record_animation.cpp
)

target_include_directories(record_animation PUBLIC /usr/local/webots/include/controller/cpp)
target_link_libraries(record_animation PUBLIC
    Controller
    CppController
    absl::time
    absl::flags
    absl::flags_parse
)
