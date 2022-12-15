load("@rules_cc//cc:defs.bzl", "cc_library")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "gnss_parser",
    deps = [
        ":data_parser",
        ":ins570d_parser",
    ],
)

cc_library(
    name = "data_parser",
    srcs = ["data_parser.cc"],
    hdrs = [
        "data_parser.h",
        "parser.h",
    ],
    deps = [
        ":ins570d_parser",
        "//cyber",
        "//modules/common/adapters:adapter_gflags",
        "//modules/common/util:message_util",
        "//modules/drivers/gnss/proto:gnss_best_pose_cc_proto",
        "//modules/drivers/gnss/proto:gnss_cc_proto",
        "//modules/drivers/gnss/proto:gnss_status_cc_proto",
        "//modules/drivers/gnss/util:gnss_util",
        "//modules/localization/proto:gps_cc_proto",
        "//modules/localization/proto:imu_cc_proto",
        "//modules/transform:transform_broadcaster",
        "@eigen",
        "@proj",
    ],
)

cc_library(
    name = "ins570d_parser",
    srcs = ["ins570d_parser.cc"],
    hdrs = [
        "ins570d_messages.h",
        "INS570D_Define.h",
        "parser.h",
    ],
    deps = [
        "//cyber",
        "//modules/common/adapters:adapter_gflags",
        "//modules/common/proto:error_code_cc_proto",
        "//modules/common/proto:geometry_cc_proto",
        "//modules/common/proto:header_cc_proto",
        "//modules/drivers/gnss/proto:config_cc_proto",
        "//modules/drivers/gnss/proto:gnss_best_pose_cc_proto",
        "//modules/drivers/gnss/proto:gnss_cc_proto",
        "//modules/drivers/gnss/proto:gnss_raw_observation_cc_proto",
        "//modules/drivers/gnss/proto:heading_cc_proto",
        "//modules/drivers/gnss/proto:imu_cc_proto",
        "//modules/drivers/gnss/proto:ins_cc_proto",
        "//modules/drivers/gnss/util:gnss_util",
    ],
)

cpplint()
