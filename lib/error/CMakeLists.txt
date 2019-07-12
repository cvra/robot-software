add_library(error
    error.c
)

target_include_directories(error PUBLIC include)

cvra_add_test(TARGET error_test SOURCES 
    tests/error.cpp
    DEPENDENCIES
    error
)
