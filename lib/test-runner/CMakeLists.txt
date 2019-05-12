
add_library(test-runner
    main.cpp
)

target_link_libraries(test-runner
    CppUTest
    CppUTestExt
)
