# ROS RIGOR: Reducing Compute Strain for running ROS on Low-Power Devices

![Project Hermes with TF Frames](/assets/Demo.gif "Project Hermes")

__The following is a living document to show the timeline, goals, and progress of the "ROS Rigor" research project__



# Goal 1: Design a Robot to Utilize

**Note** this robot is the result of work I did in my undergrad, with all parts being bought by me. I will happily share the design files, but the physical robot will remain with me.

![Project Hermes](/assets/Robot.png "Project Hermes")

![Project Hermes with Pivot](/assets/pivot.gif "Project Hermes")

# Goal 2: Create a Dev Environment

## Conclusion

The following will create a Ros2 Jammy, Ubuntu Noble, Gazebo Harmonic image for you to use

```
cd local_env
docker compose up -d
```

# Goal 3: Run the Gazebo/Rviz Environment

![GazeboRviz](/assets/GazeboRviz.png "GazeboRviz")

## Notes:

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

ros2 launch ros_gz_sim gz_sim.launch.py gz_args:=empty.sdf


cd /opt/ros/jazzy/share

root@6ff27fb03f10:/Robots/hermes_robot/launch# ros2 launch gazebo.launch.py 

https://github.com/gazebosim/ros_gz/blob/ros2/ros_gz_bridge/launch/ros_gz_bridge.launch.py

ros2 run ros_gz_bridge parameter_bridge --ros-args -p config_file:=ScanConfig.yaml 

## Conclusion

The following can be run, and will share the RVIZ and Gazebo windows outside of your container for viewing

```
xhost +local:docker
docker exec -it ros2 bash
cd /Robots/hermes_robot
colcon build
source install/setup.bash
cd launch
ros2 launch gazebo.launch.py 
```

# Goal 4: Link Gazebo and RVIZ

## Notes:

Currently running into issues with the ros_gz_bridge not persisting the joint_states across the gz and ros environments.

https://discordapp.com/channels/1077825543698927656/1303391211435851899


AH! Gazebo needs to be the ground source of truth for physics, and RVIZ used as just a visulaizer
https://github.com/gazebosim/ros_gz_project_template/blob/main/ros_gz_example_bringup/launch/rrbot_setup.launch.py

https://github.com/freshrobotics/swerve-sim-container

https://www.freshconsulting.com/insights/blog/how-to-build-a-swerve-drive-robot/

https://github.com/ros/ros_tutorials/tree/jazzy/turtlesim

ros2 launch robot.launch.py

https://gazebosim.org/docs/harmonic/moving_robot/

https://gazebosim.org/api/sim/8/tutorials.html

https://github.com/gazebosim/gz-sim/tree/gz-sim8/examples/plugin

https://github.com/gazebosim/gz-sim/tree/gz-sim8/examples/plugin/command_actor

# Goal 5: Create a Simulated Gazebo Environment

gz topic -l

- Publish to a topic:

gz topic -e -t /drive/4/command

ros2 topic pub /swerve_a/rqst_pivot_direction std_msgs/msg/Float64 "{data: 0}"


# Goal 6: Write basic Robot Control Code 



# Goal 7: Run that code on Raspberry Pi, with simulated Sensors

# Goal 8: Orchestrate Testing from PC to Pi 


# Goal 9: Compile Data from Testing to Come to a Conclusion

# Goal 10: Write Thesis
