cmake_minimum_required(VERSION 3.10.2 FATAL_ERROR)

project(SwerveDrivePlugin)

# Find required packages
find_package(gz-cmake3 REQUIRED)
find_package(gz-plugin2 REQUIRED COMPONENTS register)
find_package(gz-sim8 REQUIRED)
# find_package(gazebo_ros REQUIRED)
# find_package(rclcpp REQUIRED)
# find_package(std_msgs REQUIRED)

# Set Gazebo plugin version
set(GZ_PLUGIN_VER ${gz-plugin2_VERSION_MAJOR})

# Declare the library
add_library(SwerveDrivePlugin SHARED SwerveDrivePlugin.cc)
set_property(TARGET SwerveDrivePlugin PROPERTY CXX_STANDARD 17)

# Include directories
target_include_directories(SwerveDrivePlugin PUBLIC
  ${rclcpp_INCLUDE_DIRS}
  ${std_msgs_INCLUDE_DIRS}
)

# Link libraries
target_link_libraries(SwerveDrivePlugin PRIVATE
  gz-plugin${GZ_PLUGIN_VER}::gz-plugin${GZ_PLUGIN_VER}
  gz-sim8::gz-sim8
  ${rclcpp_LIBRARIES}
  ${std_msgs_LIBRARIES}
)

# Specify necessary compile definitions for compatibility
target_compile_definitions(SwerveDrivePlugin PRIVATE "RCLCPP_ENABLE_EXPERIMENTAL_TARGETS")
