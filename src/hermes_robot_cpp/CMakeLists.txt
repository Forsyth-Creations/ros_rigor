cmake_minimum_required(VERSION 3.8)
project(hermes_robot_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find required dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)  # Find rclcpp package
find_package(std_msgs REQUIRED)  # Find std_msgs package
find_package(geometry_msgs REQUIRED)  # Find geometry_msgs package
find_package(nav_msgs REQUIRED)  # Find nav_msgs package
find_package(sensor_msgs REQUIRED)  # Find sensor_msgs package
find_package(tf2_ros REQUIRED)  # Find tf2_ros package
find_package(tf2_msgs REQUIRED)  # Required for TF message types
find_package(tf2_geometry_msgs REQUIRED)

# Create executable
add_executable(hermes_robot_cpp src/hermes_robot_cpp.cpp)
add_executable(listener src/listener.cpp)
add_executable(talker src/talker.cpp)
add_executable(swerve_module src/swerve_module.cpp)

# Link dependencies using ament_target_dependencies
ament_target_dependencies(
  hermes_robot_cpp
  rclcpp
  std_msgs
  geometry_msgs
  nav_msgs
  sensor_msgs
  tf2_ros
  tf2_msgs
  tf2_geometry_msgs
)

ament_target_dependencies(
  listener
  rclcpp
  std_msgs
)

ament_target_dependencies(
  talker
  rclcpp
  std_msgs
)

ament_target_dependencies(
  swerve_module
  rclcpp
  std_msgs
  geometry_msgs
  sensor_msgs
)

target_include_directories(hermes_robot_cpp PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_include_directories(swerve_module PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# Set C++ standard requirements
target_compile_features(hermes_robot_cpp PUBLIC cxx_std_17)  # Require C++17

# Install executable
install(TARGETS hermes_robot_cpp listener talker swerve_module
  DESTINATION lib/${PROJECT_NAME})

# install the launch files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
  FILES_MATCHING PATTERN "*.launch.py"
)


# Handle testing dependencies
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # Skip linter checks for now
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
