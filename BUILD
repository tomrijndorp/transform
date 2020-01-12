cc_library(
    name = 'transform',
    hdrs = ['include/transform.hpp'],
    visibility = ["//visibility:public"],
)

cc_test(
    name = 'test_transform',
    srcs = [
        'test/test_transform.cpp',
    ],
    deps = [
        'transform',
        '@googletest//:gtest_main'
    ],
)