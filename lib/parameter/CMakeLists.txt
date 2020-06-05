add_library(parameter
parameter.c
parameter_msgpack.c
parameter_print.c
)

target_include_directories(parameter PUBLIC include)


target_link_libraries(parameter
    cmp_mem_access
    cmp
    cmp_mem_access
    )


if(NOT ${CMAKE_CROSSCOMPILING})
    add_library(parameter_port_dummy
        tests/parameter_port_dummy.cpp
    )

    target_link_libraries(parameter_port_dummy parameter)
endif()


cvra_add_test(TARGET parameter_test SOURCES 
    tests/parameter_test.cpp
    tests/parameter_types_test.cpp
    tests/parameter_print_test.cpp
    tests/msgpack_test.cpp
    DEPENDENCIES
    parameter
    parameter_port_dummy
)
