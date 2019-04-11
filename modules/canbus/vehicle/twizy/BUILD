load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "twizy_vehicle_factory",
    srcs = [
        "twizy_vehicle_factory.cc",
    ],
    hdrs = [
        "twizy_vehicle_factory.h",
    ],
    deps = [
        ":twizy_controller",
        ":twizy_message_manager",
        "//modules/canbus/vehicle:abstract_vehicle_factory",
    ],
)

cc_library(
    name = "twizy_message_manager",
    srcs = [
        "twizy_message_manager.cc",
    ],
    hdrs = [
        "twizy_message_manager.h",
    ],
    deps = [
        "//modules/canbus/common:canbus_common",
        "//modules/canbus/proto:canbus_proto",
        "//modules/canbus/vehicle/twizy/protocol:canbus_twizy_protocol",
        "//modules/drivers/canbus/can_comm:message_manager_base",
    ],
)

cc_library(
    name = "twizy_controller",
    srcs = [
        "twizy_controller.cc",
    ],
    hdrs = [
        "twizy_controller.h",
    ],
    deps = [
        ":twizy_message_manager",
        "//modules/drivers/canbus/can_comm:can_sender",
        "//modules/drivers/canbus/can_comm:message_manager_base",
        "//modules/drivers/canbus/common:canbus_common",
        "//modules/canbus/proto:canbus_proto",
        "//modules/canbus/vehicle:vehicle_controller_base",
        "//modules/canbus/vehicle/twizy/protocol:canbus_twizy_protocol",
    ],
)

cpplint()