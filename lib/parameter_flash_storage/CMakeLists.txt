add_library(parameter_flash_storage
parameter_flash_storage.c
)

target_include_directories(parameter_flash_storage PUBLIC include)

target_link_libraries(parameter_flash_storage
    parameter
    crc
)

cvra_add_test(TARGET parameter_flash_storage_test SOURCES 
    tests/flash_mock.cpp
    tests/parameter_flash_storage.cpp
    DEPENDENCIES
    parameter_flash_storage
    parameter_port_dummy
)

add_library(parameter_flash_f4
    flash_f4.c
)

target_include_directories(parameter_flash_f4 PUBLIC include)

add_library(parameter_flash_f3
    flash_f3.c
)

target_include_directories(parameter_flash_f3 PUBLIC include)
