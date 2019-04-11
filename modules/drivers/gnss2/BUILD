load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_binary(
    name = "libheading_gnss.so",
    linkopts = ["-shared"],
    linkstatic = False,
    deps = [":heading_gnss_lib"],
)

cc_library(
    name = "heading_gnss_lib",
    srcs = [
        "heading_gnss.cc",
    ],
    hdrs = [
        "heading_gnss.h",
    ],
    deps = [
        "//cyber",
        "//modules/drivers/gnss2/proto:heading_gnss_cc_proto",
	"//modules/drivers/gnss2/libsbp:libsbp",
	"//modules/drivers/gnss2/third_party:rtcm",
	"//modules/drivers/gnss2/src:TCPClient",
    ],
)

cpplint()
