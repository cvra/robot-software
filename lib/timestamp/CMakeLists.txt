add_library(timestamp
timestamp.c
)

target_include_directories(timestamp PUBLIC include)

cvra_add_test(TARGET timestamp_test SOURCES 
    timestamp_test.cpp
    DEPENDENCIES
    timestamp
)

if (CMAKE_CROSSCOMPILING)
    add_library(timestamp_stm32
        timestamp_stm32.c
    )

    target_link_libraries(timestamp_stm32 timestamp chibios)
endif()
