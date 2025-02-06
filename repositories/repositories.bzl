load("@bazel_tools//tools/build_defs/repo:git.bzl", "git_repository")
load("@bazel_tools//tools/build_defs/repo:http.bzl", "http_archive")
load("@bazel_tools//tools/build_defs/repo:utils.bzl", "maybe")
load("@bazel_tools//tools/build_defs/repo:git.bzl", "new_git_repository")

def load_ros2_rules():

    git_repository (
    name = "com_github_mvukov_rules_ros2",
    remote = "https://github.com/mvukov/rules_ros2.git",
    commit = "4099c8d0acb2f80ce8b50dd64d3081404d6ac6b8"
    )