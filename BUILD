load("//bzl:module.bzl", "isaac_app", "isaac_cc_module")

isaac_cc_module(
    name = "components",
    srcs = [
        "ManipulateImageNode.cpp",
        "ManipulateBoundingBoxesNode.cpp",
    ],
    hdrs = [
        "ManipulateImageNode.hpp",
        "ManipulateBoundingBoxesNode.hpp",
    ],
    visibility = ["//visibility:public"],
    deps = [
    ],
)

isaac_app(
    name = "distance_project",
    modules = [
        "sight",
        "//apps/distance_project:components",
        "engine_tcp_udp",
    ],
)