load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_binary(
    name = "libapril_tags.so",
    linkopts = ["-shared"],
    linkstatic = False,
    deps = [":april_tags_lib"],
)

cc_library(
    name = "april_tags_lib",
    srcs = [
        "april_tags.cc",
    ],
    hdrs = [
        "april_tags.h",
    ],
    linkopts = [
        "-lopencv_core",
        "-lopencv_highgui",
        "-lopencv_imgproc",
    ],
    deps = [
        "@apriltags_lib//:apriltags_l",
        #"@apriltags_headers//:apriltags_h",
        "//cyber",
        "//modules/drivers/apriltags/proto:april_tags_cc_proto",
       "//modules/drivers/proto:sensor_proto",
        "@opencv2//:core",
        "@opencv2//:highgui",
    ],
)

cpplint()
