add_library(cmp
cmp.c

)

target_include_directories(cmp PUBLIC include)

cvra_add_test(TARGET cmp_test SOURCES 
    test/buf.c
    test/utils.c
    test/test.c
    DEPENDENCIES
    cmp
)
