add_library(msgbus
    messagebus.c
)

target_include_directories(msgbus PUBLIC include)

cvra_add_test(TARGET msgbus_test SOURCES
    tests/mocks/synchronization.cpp
    tests/atomicity.cpp
    tests/msgbus.cpp
    tests/signaling.cpp
    tests/foreach.cpp
    tests/watchgroups.cpp
    tests/new_topic_callbacks.cpp
    tests/test_cpp_interface.cpp
    DEPENDENCIES
    msgbus
)

if(${CMAKE_CROSSCOMPILING})
    add_library(msgbus_chibios
        examples/chibios/port.c
    )
    target_link_libraries(msgbus_chibios msgbus chibios)
else()
    find_package (Threads)

    add_library(msgbus_posix
        examples/posix/port.c
    )

    target_link_libraries(msgbus_posix
       msgbus
       ${CMAKE_THREAD_LIBS_INIT}
    )

    add_executable(msgbus_demo
        examples/posix/demo.c
    )
    target_link_libraries(msgbus_demo
        msgbus
        msgbus_posix
    )

    add_executable(msgbus_demo_watchgroups
        examples/posix/demo_watchgroups.c
    )

    target_link_libraries(msgbus_demo_watchgroups 
        msgbus
        msgbus_posix
    )
endif()
