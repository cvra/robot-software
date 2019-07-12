add_library(quadramp
    quadramp.c
)

target_include_directories(quadramp PUBLIC include)

cvra_add_test(TARGET quadramp_test SOURCES 
    tests/quadramp.cpp
    DEPENDENCIES
    quadramp
)
