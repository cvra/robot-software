add_library(trace
    trace.c
)

target_include_directories(trace PUBLIC include)

add_library(trace_platform_mocks
    tests/trace_mocks.c
    tests/trace_points.c
)
target_link_libraries(trace_platform_mocks trace)

cvra_add_test(TARGET trace_test SOURCES 
    tests/trace_test.cpp
    DEPENDENCIES
    trace
    trace_platform_mocks
)

