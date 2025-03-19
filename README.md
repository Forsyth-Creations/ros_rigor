![Project Hermes Full](/assets/DemoAll.gif "Project Hermes")
![Project Hermes with Pivot](/assets/pivot.gif "Project Hermes")

# Starting the Project

**Disclaimer: This work was accomplished with a PC running Ubuntu 24.04, With a 24 core AMD CPU. You must also have Docker installed. Gazebo Harmonic (gz sim 8), ROS2 Jazzy**

```
xhost +local:docker
docker compose up -d
docker exec -it rigor bash
make all
source setup_env.sh
ros2 launch hermes_robot all.launch.py 
```

# Overview of the Project

More of the specifics on timelines and goals can be found [here](/LIVING_DOC.md)

# High Level Focus

To keep this document understandable, each goal represents a milestone I'd like to achieve. Once all the milestones are complete, the research is complete. The high-level goal is to be able to run a simulated environment on my PC that a Raspberry Pi can interact with. I will vary the environment and the ROS params that the Pi consumes to simulate how it behaves/achieves autonomy in a simulated space. The hypothesis is that, for low risk scenarios, you can reduce the frequency of data aquisition. A "low risk" scenario is one with few other dynamic actors. "High Risk" would be the opposite: greater sampling of the area. 

While this idea isn't novel on the face, I haven't found research into to what degree of risk requires what degree of sampling. Most companies right now just use high same rates as a rule, which can decrease the lifespan of their products. I want to know if there is a more dynamic way of doing it.

Note: Each goal section will have notes, thoughts, and conclusions. The idea is that anyone could restart this research by simply following the conclusion blocks.

# Understanding the File Structure

## src/hermes_robot

This set of files contains the basic files to launch the robot and it's simulation environment (check the launch directory). There are two launch files:

- all.launch.py
    - This launches all the packages to run the sim, including the "full_robot" which is listed in the `src/hermes_swerve_module` directory
- robot.launch.py
    - Will soon (this is a work in progress) be used to deploy the ROS nodes onto a real-world robot

This directory also includes the robot node, which is a combination of four swerve_module nodes

## src/hermes_robot_description

This directory keeps track of all the files needed to render the robot into both RVIZ and Gazebo (namely, the robot description). Note that the launch files in this directory are as follows:

- bridge.launch.py: the ROS/Gazebo bridge
- robot.launch.py: The robot description launch file, including the robot state publisher (for pose, I believe)
- rviz.launch.py: A launch file for rviz
- world.launch.py: The Gazebo world to launch

## src/hermes_swerve_module

- full_robot.launch.py: A launch file to launch four swerve drive modules
- single_module.launch.py: launches a single module


## src/web_controller

- webapp.launch.py: launches the web controller