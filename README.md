# ros_rigor
A system for making efficient systems that reduce compute load


# Starting up the Workspace
docker compose run --build rigor

# Useful commands

```
colcon build
source install/setup.bash
ros2 pkg list | grep hermes
ros2 launch urdf_tutorial display.launch.py model:=/Robots/hermes_robot/urdf/hermes_robot.urdf