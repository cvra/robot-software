cvra_add_test(TARGET version_test
    SOURCES tests/test_version.cpp
    DEPENDENCIES version
)

add_custom_command(COMMAND ./version.sh \"${CVRA_BOARD_NAME}\" ${CMAKE_BINARY_DIR}/version.c
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
    OUTPUT ${CMAKE_BINARY_DIR}/version.c
    COMMENT "Generating git version"
)

add_library(version ${CMAKE_BINARY_DIR}/version.c)
target_include_directories(version INTERFACE include)
