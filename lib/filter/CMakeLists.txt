add_library(filter
iir.c
basic.c

)

target_include_directories(filter PUBLIC include)

cvra_add_test(TARGET filter_test SOURCES 
    tests/iir_test.cpp
    tests/basic_test.cpp
    DEPENDENCIES
    filter
)