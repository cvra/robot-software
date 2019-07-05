add_library(trace
trace.c
)

target_include_directories(trace PUBLIC include)



cvra_add_test(TARGET trace_test SOURCES 
    tests/trace_test.cpp
    tests/trace_points.c
    DEPENDENCIES
    trace
)