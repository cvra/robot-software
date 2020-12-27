project(goap)
cmake_minimum_required(VERSION 3.10)

add_executable(example
    example.cpp
)

target_include_directories(example PUBLIC include)

add_executable(tests
    tests/goap_internals.cpp
    tests/goap_test.cpp
    tests/main.cpp
)
target_include_directories(tests PUBLIC include)
target_link_libraries(tests CppUTest CppUTestExt)
