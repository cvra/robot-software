project(cvra)
cmake_minimum_required(VERSION 3.14)

# Generate a test exec given test sources and their dependencies
# Called like cvra_add_test(TARGET foo_test SOURCES foo_test.cpp DEPENDENCIES foo_lib)
function(cvra_add_test)
    set(options)
    set(oneValueArgs TARGET)
    set(multiValueArgs SOURCES DEPENDENCIES)
    cmake_parse_arguments(TEST_EXECUTABLE "${options}" "${oneValueArgs}" "${multiValueArgs}" ${ARGN})
    add_executable(${TEST_EXECUTABLE_TARGET} ${TEST_EXECUTABLE_SOURCES})

    target_link_libraries(${TEST_EXECUTABLE_TARGET} ${TEST_EXECUTABLE_DEPENDENCIES} test-runner)
    add_test(${TEST_EXECUTABLE_TARGET} ${TEST_EXECUTABLE_TARGET})
endfunction()

enable_testing()

add_subdirectory(lib)
