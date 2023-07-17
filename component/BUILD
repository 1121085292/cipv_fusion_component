load("//tools:cpplint.bzl", "cpplint")
load("@rules_cc//cc:defs.bzl", "cc_binary", "cc_library")

package(default_visibility = ["//visibility:public"])

cc_binary(
    name = "libcipv_fusion_component.so",
    linkshared = True,
    linkstatic = True,
    deps = [
        ":cipv_fusion_component_lib",
    ],
)

cc_library(
    name = "cipv_fusion_component_lib",
    srcs = ["radar_detection_component.cc"],
    hdrs = ["radar_detection_component.h"],
    alwayslink = True,    
    deps = [
        "//cyber/component",
        "//cipv_fusion_component/proto:car_state_cc_proto",
        "//cipv_fusion_component/proto:camera_detection_cc_proto",
        "//cipv_fusion_component/proto:radar_detection_cc_proto",
    ],
)
# -----------------------------------
cc_binary(
    name = "libradar_detection.so",
    linkshared = True,
    linkstatic = True,
    deps = [
        ":radar_detection_lib",
    ],
)

cc_library(
    name = "radar_detection_lib",
    srcs = ["radar_publish.cc"],
    alwayslink = True,
    deps = [
        "//cyber",
        "//cipv_fusion_component/proto:radar_detection_cc_proto",
    ],
)
# -----------------------------------------
cc_binary(
    name = "libcar_state.so",
    linkshared = True,
    linkstatic = True,
    deps = [
        ":car_state_lib",
    ],
)

cc_library(
    name = "car_state_lib",
    srcs = ["car_publish.cc"],
    alwayslink = True,
    deps = [
        "//cyber",
        "//cipv_fusion_component/proto:car_state_cc_proto",
    ],
)
# -----------------------------------------
cc_binary(
    name = "libcamera_state.so",
    linkshared = True,
    linkstatic = True,
    deps = [
        ":camera_state_lib",
    ],
)

cc_library(
    name = "camera_state_lib",
    srcs = ["camera_publish.cc"],
    alwayslink = True,
    deps = [
        "//cyber",
        "//cipv_fusion_component/proto:camera_detection_cc_proto",
    ],
)
cpplint()
