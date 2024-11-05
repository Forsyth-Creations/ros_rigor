# ros_rigor
A system for making efficient systems that reduce compute load


# Starting up the Workspace
docker compose run --build rigor

# Useful commands

Launching RVIZ to play with the robot

```
colcon build
source install/setup.bash
xhost +local:docker
ros2 pkg list | grep hermes
ros2 launch urdf_tutorial display.launch.py model:=/Robots/hermes_robot/urdf/hermes_robot.urdf
```

Launching Gazebo to see the robot

https://gazebosim.org/docs/harmonic/spawn_urdf/

gz sim empty.sdf

gz service -s /world/empty/create --reqtype gz.msgs.EntityFactory --reptype gz.msgs.Boolean --timeout 1000 --req 'sdf_filename: "/Robots/hermes_robot/urdf/hermes_robot_2.urdf", name: "urdf_model"'

# Now with ROS things

Starts it:

ros2 launch ros_gz_sim gz_sim.launch.py gz_args:=empty.sdf


cd /opt/ros/jazzy/share

root@6ff27fb03f10:/Robots/hermes_robot/launch# ros2 launch gazebo.launch.py 

https://github.com/gazebosim/ros_gz/blob/ros2/ros_gz_bridge/launch/ros_gz_bridge.launch.py

ros2 run ros_gz_bridge parameter_bridge --ros-args -p config_file:=ScanConfig.yaml 
