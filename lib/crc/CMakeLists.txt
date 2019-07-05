add_library(crc
crc32.c
crc16.c
)

target_include_directories(crc PUBLIC include)



cvra_add_test(TARGET crc_test SOURCES 
    test/crc32_test.cpp
    test/crc16_test.cpp
    DEPENDENCIES
    crc
)