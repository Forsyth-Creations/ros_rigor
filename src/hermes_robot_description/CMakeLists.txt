cmake_minimum_required(VERSION 3.5)
project(hermes_robot_description)

# Find dependencies (if needed, e.g., for ROS 2 features)
find_package(ament_cmake REQUIRED)

# Optionally, install your URDF files
install(DIRECTORY urdf/
  DESTINATION share/${PROJECT_NAME}/urdf
)

# Install project configuration files
install(
  DIRECTORY
    config/
  DESTINATION share/${PROJECT_NAME}/config
)

# Add in the mesh files
install(DIRECTORY meshes/
  DESTINATION share/${PROJECT_NAME}/meshes
)

# Add the launch files
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

ament_package()