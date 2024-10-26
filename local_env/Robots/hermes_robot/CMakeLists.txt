cmake_minimum_required(VERSION 3.5)
project(hermes_robot)

# Find dependencies (if needed, e.g., for ROS 2 features)
find_package(ament_cmake REQUIRED)

# Optionally, install your URDF files
install(DIRECTORY urdf/
  DESTINATION share/${PROJECT_NAME}/urdf
)

# Add in the mesh files
install(DIRECTORY meshes/
  DESTINATION share/${PROJECT_NAME}/meshes
)

ament_package()