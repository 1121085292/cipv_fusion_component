load("//tools:cpplint.bzl", "cpplint")
load("@rules_cc//cc:defs.bzl", "cc_binary", "cc_library")

package(default_visibility = ["//visibility:public"])

cc_binary(
    name = "libcipv_fusion_component.so",
    linkshared = True,
    linkstatic = True,
    deps = [
        ":cipv_fusion_component_lib",
        ":radar_detection_lib",
        ":car_state_lib",
        ":camera_detection_lib",
    ],
)

cc_library(
    name = "cipv_fusion_component_lib",
    srcs = ["cipv_fusion_component.cc"],
    hdrs = ["cipv_fusion_component.h"],
    alwayslink = True,    
    deps = [
        "//cyber",
        "//cipv_fusion_component/proto:car_state_cc_proto",
        "//cipv_fusion_component/proto:camera_detection_cc_proto",
        "//cipv_fusion_component/proto:radar_detection_cc_proto",
        "//cipv_fusion_component/proto:cipv_fusion_cc_proto",
    ],
)

cc_library(
    name = "radar_detection_lib",
    srcs = ["radar_detection_component.cc"],
    hdrs = ["radar_detection_component.h"],
    alwayslink = True,
    deps = [
        "//cyber",
        "//cipv_fusion_component/proto:radar_detection_cc_proto",
    ],
)

cc_library(
    name = "car_state_lib",
    srcs = ["car_state_component.cc"],
    hdrs = ["car_state_component.h"],
    alwayslink = True,
    deps = [
        "//cyber",
        "//cipv_fusion_component/proto:car_state_cc_proto",
    ],
)
# # -----------------------------------------
cc_library(
    name = "camera_detection_lib",
    srcs = ["camera_detection_component.cc"],
    hdrs = ["camera_detection_component.h"],
    alwayslink = True,
    deps = [
        "//cyber",
        "//cipv_fusion_component/proto:camera_detection_cc_proto",
    ],
)

cpplint()