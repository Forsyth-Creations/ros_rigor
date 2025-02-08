load("@bazel_tools//tools/build_defs/repo:git.bzl", "git_repository")
load("@bazel_tools//tools/build_defs/repo:http.bzl", "http_archive")
load("@bazel_tools//tools/build_defs/repo:utils.bzl", "maybe")
load("@bazel_tools//tools/build_defs/repo:git.bzl", "new_git_repository")

def load_ros2_rules():

    git_repository (
    name = "com_github_mvukov_rules_ros2",
    remote = "https://github.com/mvukov/rules_ros2.git",
    commit = "d2a58aecad179d4664303377bc813dfa755723fb"
    )


def load_ros2_messages():

    git_repository(
    name = "ros2_sensor_msgs",
    remote = "https://github.com/ros2/common_interfaces.git",
    branch = "jazzy",  # Adjust for your ROS 2 version
    )