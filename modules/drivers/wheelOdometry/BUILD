load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_binary(
    name = "lib_wheel_odometry.so",
    linkopts = ["-shared"],
    linkstatic = False,
    deps = [":wheel_odometry_lib"],
)

cc_library(
    name = "wheel_odometry_lib",
    srcs = [
        "wheel_odometry.cc",
    ],
    hdrs = [
        "wheel_odometry.h",
        "cArduino.h",
    ],
    deps = [
        "//cyber",
        "//modules/drivers/wheelOdometry/proto:wheel_odometry_cc_proto",
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
