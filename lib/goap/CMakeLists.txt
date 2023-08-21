add_library(goap INTERFACE)
target_include_directories(goap INTERFACE include)

cvra_add_test(TARGET goap_test SOURCES
    tests/goap_internals.cpp
    tests/goap_test.cpp
    DEPENDENCIES
    goap
)

add_executable(goap_example
    example.cpp
)

target_link_libraries(goap_example goap)
