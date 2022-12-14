load("@rules_cc//cc:defs.bzl", "cc_library")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "gnss_stream",
    deps = [
        ":raw_stream",
        ":serial_stream",
    ],
)


cc_library(
    name = "raw_stream",
    srcs = ["raw_stream.cc"],
    hdrs = ["raw_stream.h"],
    deps = [
        ":serial_stream",
        ":stream",
        "//cyber",
        #"//modules/canbus/proto:chassis_cc_proto",
        "//modules/common/adapters:adapter_gflags",
        "//modules/common/configs/proto:vehicle_config_cc_proto",
        "//modules/common/proto:drive_state_cc_proto",
        "//modules/common/proto:vehicle_signal_cc_proto",
        "//modules/common/util:message_util",
        "//modules/drivers/gnss/parser:gnss_parser",
        "//modules/drivers/gnss/proto:gnss_status_cc_proto",
        "//modules/drivers/gnss/proto:heading_cc_proto",
        "//modules/drivers/gnss/proto:imu_cc_proto",
        "//modules/drivers/gnss/util:gnss_util",
        "@com_google_absl//absl/strings",
    ],
)

cc_library(
    name = "serial_stream",
    srcs = ["serial_stream.cc"],
    deps = [
        ":stream",
        "//cyber",
        "//modules/drivers/gnss/util:gnss_util",
    ],
)


cc_library(
    name = "stream",
    hdrs = ["stream.h"],
    deps = [
        "//cyber",
        "//modules/drivers/gnss/util:gnss_util",
    ],
)

cpplint()
