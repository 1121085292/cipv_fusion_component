load("//tools/install:install.bzl", "install", "install_src_files")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

filegroup(
    name = "conf",
    srcs = [
        ":cipv.dag",
        ":cipv.launch",
    ],
)

install(
    name = "install",
    data = [
        "//cipv_fusion_component/proto:camera_detection_cc_proto",
        "//cipv_fusion_component/proto:radar_detection_cc_proto",
        "//cipv_fusion_component/proto:car_state_cc_proto",
        "cyberfile.xml",
    ],
    data_dest = "cipv_fusion_component",
    library_dest = "cipv_fusion_component/lib",
    library_strip_prefix = ["component", "proto"],
    targets = [
        "//cipv_fusion_component/component:libcipv_fusion_component.so",
        "//cipv_fusion_component/proto:camera_detection_cc_proto",
        "//cipv_fusion_component/proto:radar_detection_cc_proto",
        "//cipv_fusion_component/proto:car_state_cc_proto",
    ],
    deps = [
        "pb_headers",
        ":dag",
        ":launch",
    ],
)

install(
    name = "pb_headers",
    data = [
        "//cipv_fusion_component/proto:camera_detection_cc_proto",
        "//cipv_fusion_component/proto:radar_detection_cc_proto",
        "//cipv_fusion_component/proto:car_state_cc_proto",
    ],
    data_dest = "cipv_fusion_component/include"
)

install(
    name = "dag",
    data = [":cipv.dag"],
    data_dest = "cipv_fusion_component/dag"
)

install(
    name = "launch",
    data = [":cipv.launch"],
    data_dest = "cipv_fusion_component/launch"
)

install_src_files(
    name = "headers",
    src_dir = ["component"],
    dest = "cipv_fusion_component/include",
    filter = "*.h",
)

install_src_files(
    name = "install_src",
    src_dir = ["."],
    dest = "cipv_fusion_component/component",
    filter = "*",
    deps = [
        ":headers"
    ],
)