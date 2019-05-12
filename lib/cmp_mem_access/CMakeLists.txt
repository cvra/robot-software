add_library(cmp_mem_access
cmp_mem_access.c
)

target_include_directories(cmp_mem_access PUBLIC include)

target_link_libraries(cmp_mem_access
    cmp
    )

cvra_add_test(TARGET cmp_mem_access_test SOURCES 
    cmp_mem_access_test.cpp
    DEPENDENCIES
    cmp_mem_access
)
