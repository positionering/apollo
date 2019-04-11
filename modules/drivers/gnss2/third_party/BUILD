load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "rtcm",
    srcs = [
    	"sbp.c",
    	"edc.c",
        "novatel.c",
        "rcvraw.c",
        "rtcm.c",
        "rtcm3.c",
        "rtkcmn.c",
    ],
    hdrs = [
    "rtklib.h",
    ],
    deps = [
    	"//modules/drivers/gnss/libsbp:libsbp",
    ],
)
