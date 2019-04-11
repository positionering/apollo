load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_binary(
    name = "lib_fake_gps.so",
    linkopts = ["-shared"],
    linkstatic = False,
    deps = [":fake_gps_lib"],
)

cc_library(
    name = "fake_gps_lib",
    srcs = [
        "fake_gps.cc",
    ],
    hdrs = [
        "fake_gps.h",
    ],
    deps = [
        "//cyber",
        "//modules/drivers/fakeGps/proto:fake_gps_cc_proto",
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
