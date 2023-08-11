load("@rules_cc//cc:defs.bzl", "cc_library")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "radard",
    srcs = ["radard.cc",],
    hdrs = ["radard.h"],
    deps = [
        "//cyber",
        ":fast_cluster",
        ":track",
    ],
)

cc_library(
    name = "fast_cluster",
    srcs = ["fast_cluster.cc",],
    hdrs = ["fast_cluster.h"],
    deps = [
        "//cipv_fusion_component/src:fast_cluster_R_dm",
    ],
)

cc_library(
    name = "fast_cluster_R_dm",
    srcs = ["fast_cluster_R_dm.hpp"],
    deps = [
        "//cipv_fusion_component/src:fast_cluster_dm",
    ],
)

cc_library(
    name = "fast_cluster_dm",
    srcs = ["fast_cluster_dm.hpp"],
)

cc_library(
    name = "kalman_filter",
    srcs = ["kalman_filter.cc"],
    hdrs = ["kalman_filter.h"],
)

cc_library(
    name = "track",
    srcs = ["track.cc",],
    hdrs = ["track.h"],
    deps = [
        "//cipv_fusion_component/src:meta",
        "kalman_filter",
    ],
)

cc_library(
    name = "meta",
    hdrs = ["meta.h"],
    deps = [
        "//cipv_fusion_component/proto:car_state_cc_proto",
        "//cipv_fusion_component/proto:camera_detection_cc_proto",
        "//cipv_fusion_component/proto:radar_detection_cc_proto",
        "//cipv_fusion_component/proto:cipv_fusion_cc_proto",
    ],
)
