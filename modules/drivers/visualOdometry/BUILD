load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_binary(
    name = "lib_visual_odometry.so",
    linkopts = ["-shared"],
    linkstatic = False,
    deps = [":visual_odometry_lib"],
)

cc_library(
    name = "visual_odometry_lib",
    srcs = [
        "visual_odometry.cc",
    ],
    hdrs = [
        "visual_odometry.h",
        "cArduino.h",
    ],
    deps = [
        "//cyber",
        "//modules/drivers/visualOdometry/proto:visual_odometry_cc_proto",
        "//modules/drivers/gnss/proto:gnss_proto",
        "//modules/localization/proto:gps_proto",
        "//modules/localization/proto:imu_proto",
        "//modules/transform:transform_broadcaster_lib",
        "@eigen",
        "@proj4",
        "//modules/common/adapters:adapter_gflags",
        "//modules/drivers/apriltags/proto:april_tags_cc_proto",
    ],
)

cpplint()
