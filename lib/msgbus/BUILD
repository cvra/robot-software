cc_library(
    name = "msgbus",
    srcs = [
        "messagebus.c",
        "messagebus.h",
        "messagebus_cpp.hpp",
    ],
    hdrs = ["messagebus.h"],
    include_prefix = "msgbus",
    visibility = ["//visibility:public"],
    # TODO: Add ChibiOS port
    deps = select({
        "//conditions:default": [":msgbus-pthread"],
    }),
)

cc_test(
    name = "msgbus-test",
    size = "small",
    srcs = glob([
        "tests/*.cpp",
        "tests/mocks/*.cpp",
        "tests/mocks/*.hpp",
    ]),
    deps = [
        ":msgbus",
        "@ch_cvra_test_runner//:main",
    ],
)

cc_library(
    name = "msgbus-pthread",
    srcs = [
        "examples/posix/port.c",
        "messagebus.h",
    ],
    hdrs = ["examples/posix/port.h"],
    linkopts = ["-pthread"],
)

cc_binary(
    name = "msgbus-demo",
    srcs = ["examples/posix/demo.c"],
    deps = [":msgbus"],
)

cc_binary(
    name = "msgbus-demo-watchgroups",
    srcs = ["examples/posix/demo_watchgroups.c"],
    deps = [":msgbus"],
)
