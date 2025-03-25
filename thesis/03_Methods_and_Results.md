# Methods and Results

## Introduction

This section details the methods used to develop the swerve drive robot in ROS2, including the development of the robot model, the implementation of the slam_toolbox and Nav2 stack, and the testing of the robot in simulation. The section is divided into several subsections, each detailing a specific aspect of the development process. The process unfolds chronologically as the researcher experienced them. Many of these steps, especially when divided on the typical hardware/software line, can be run in parallel. To better help your understanding, an anycdote is applied to each minor section on the division. Note that all source code and models are available on the GitHub repository for this project; the link is provided in the appendix. As this work is ongoing, attached to the Github link is a specific commit hash. You may use this to check out the code at the time of this writing.

## Development of the Model

_Prodomidently hardware_

To develop the model, one software solution stood above the rest: Solidworks. Its toolchain, though a plugin, let's you export to URDF, which is the standard for ROS2. The model was developed with the following considerations in mind:

- The frames of the robot must have the following hierarchy:
    - base_frame
        - base_link
            - WheelPivotA
                - WheelDriveA
            - WheelPivotB
                - WheelDriveB
            - WheelPivotC
                - WheelDriveC
            - WheelPivotD
                - WheelDriveD
            - camera_pivot
                - camera_link
            - IMU Link

- The X Axis is the most crucial axis to align: for Nav2, **the X axis of the base_link must be aligned with the forward motion of the robot**. This is the default for the URDF exporter, so it is important to ensure that the model is aligned with the X axis in Solidworks.
- For the pivots to function appropriately, the pivot axis must be aligned on their Z axis. Otherwise you risk improper or lopsided movement of the modules

When developing the model, the best approach is the design the model in full, developing submodules where available. Following this, you can proceed with adding points and reference axis. Remember, for the sake of simplified axis, try and keep all your axis aligned with one another. This will make it easier to export to URDF, and easier to debug later. Since this is foundational to the structure of the robot, and changing it will require work, it is best to get this right the first time.

![TF Frames](/assets/Goal6/TF_Frames_Example.png "Title")

![Reference Frames](/assets/Goal6/Example_Solidworks_Frame.png)

Butler Lampson, one of the first scientists to envision the modern personal computer in 1972 once said that "All problems in computer science can be solved by another level of indirection". However, it is the belief of this researcher that humans opperate along the inverse of this principle. As such, while there are other tools better adapted to generate launch files for robot descriptions (in this case the generated URDF) for specifically ROS2, adding another tooling into the production process is uneccsary by leveraging the URDF export plugin for Solidworks. 

## Exporting The Artifacts of the URDF, and its Common Structures

_Predominantly hardware_

Upon exporting the URDF, you will find the following file structure:

![Reference Frames](/assets/Goal6/Exported_Files_From_Solidworks.png)

Note that this structure is consistent with standard description files for ROS1. However.

When preparing the URDF, there are a few considerations to make. First and foremost, the key framework you are setting up is a link/joint relationship. Links are coordinated through joints, with links typically having some form of mesh or solid that is rendered. This relationship is seen in figure X:

![Reference Frames](/assets/Goal6/Example_URDF.png)

Within the visual, note that the link and joint, despite being closely related, do not share the same name. This is important, as the URDF will compile incorrectly if the names are the same. The joint and link should operate in a parent/child relationship. Additionally, note that Solidworks has also added the mass and inertia tags. Learned from expereince, the Gazebo physics engine is particular about these values being calculated correctly. As was a consequence in simulation, you will find the robot may flip over or behave irreadically if you attempt to edit these values manually.

## Adapting the model to be URDF Compliant: Understanding the URDF File

_Both Hardware and Software_

Despite the above warning, it should be noted that massless links and plugins do not produce a problem for the physics engine when added into the URDF. For keen eyes, you will notice that there is no native way within this export tool to define the necessary Gazebo plugins to allow for the robot to be controlled. This is a critical step, as the robot will not be able to be controlled without these plugins, nor will you be able to simulate the sensor data.

![Solidworks Export](/assets/Goal6/URDF_Export_Window_2.png)

As an additional post-processing step, you will need to add your sensors and controllers to the URDF. This is done through the use of the gazebo tags. The following is an example of the camera sensor:

![Solidworks Export](/assets/Goal6/URDF_Plugins.png)

Finally, once you have prepared this, change the file extension name to .urdf.xacro. This is critical because, on launch, this file is converted to both and URDF and an SDF so that both ROS and Gazebo can consume the proper data for rendering and simulation.

## Writing a Launch File to "Spawn" the model in Gazebo

_Predominantly software_

The approach used within this paper deviates from the typical Gazebo workflow by seperating out the world URDF from the robot URDF. In this way, we can leverage one xacro file that can be converted to both SDF and URDF, without having complicate it by wrapping it in world-file-related tags. This is done by adapting the launch file to start Gazebo, generate a world, then spawn in the robot at a given Cartesian coordinate. This paper also introduces a different approach to how the ROS2 launch files are written. Favoring a more modular approach, the launch file is broken down into smaller components, each with a specific purpose and each written in their own seperate laucnh file. Upon wanting to bring up a sequence of services, you can either run each launch file individually, or run the main launch file that will bring up all the services in the correct order. This is a more modular approach, and allows for easier debugging and testing of the robot. In accordance with this, this paper has a "world" launch file (world.launch.py) and a "robot" launch file (robot.launch.py). You can also bring up all the services by leveraging the main launch file (all.launch.py).

## Writing a World file for the Robot to Navigate in Gazebo

_Predominantly software_

The world file is a critical component to validating the sensors on a robot. It is the first baseline you will use to determine if your algorithms are accurate. In the same way the AI community is experiencing the "Garbage in Garbage out" problem, the robotics community is experiencing the same. If your world file is inconsistent with the problems you are facing, then it is worthless to your robotics development. As such, at the outset of this project, an initial world was developed to test the robot it. It consisted of a room with two obstacles, with an additional small room that was meant to act as a hallway. This worked great for initial testing. However, as the project progressed, it was noted that points within the laser scan were drifing. Initially, this was assumed to be related to the SLAM, as there were very few features within the simple room to latch onto. Upon this discovery, the world file was updated with a example from the community: a factor floor, with obstacles to avoid, to better match the use case and environment for the robot. However, even upon changing the world, the drift of the points and subsequent high-error on the maps led to the discovery that the odom calculation was incorrect, which resulted in an incorrect TF transform from the base_link to the odom frame. This was corrected, and the robot was able to navigate the world with a high degree of accuracy.

Note below the first world leveraged compared to the updated world:


![Solidworks Export](/assets/Goal6/World_laser_drift.png)

![Solidworks Export](/assets/Goal6/Updated_world.png)

## Writing a ROS Controller Node for Swerve Drive Robotics

_Predominantly software_

Most standard ROS controller nodes for robots reflect the work completed for the standard "differential drive" robot, which consists of two wheels. This work is contained in (MAR3) and is continually evolving. In the past, these were written in C to provide speed. I believe this is the correct course of action, as it maintains a focus on runtime speed, something critical in robotics. However, for readibility and for this paper to function as a first draft for a full swerve drive controller, we have opted to use Python. 

Unlike the differential drive robot, the swerve drive robot is unique in that it has four wheels, each of which can rotate independently. This requires a unique controller node to manage the movement of the robot. The controller node is responsible for taking in the velocity commands from the user, and converting them into the appropriate wheel velocities. This is done by calculating the inverse kinematics of the robot, which is a mathematical model that describes the relationship between the wheel velocities and the robot's linear and angular velocities. Upon recieving this data, in the standard convention of the /cmd_vel topic, the controller node will publish commands to the swerve drive modules, which are themselves ROS nodes. In an attempt to keep the code base clean, the modules run seperate computations for PID controls. This is done to ensure that the controller node is not bogged down with computations, and can focus on the primary task of converting the velocity commands into wheel velocities.

Additionally, when considering your inputs, the goal of this research was to integrate the NAV2 software stack. A critical observation to be made about the topic publication is that NAV2 has a default /nav_cmd_vel topic, which publishes an X heading velocity and an angular twist velocity for the z axis. However, since this robot can move in any direction, the controller node must convert this data into the appropriate wheel velocities. This is done by leveraging the Pythagorian Therom, converting the single X magnitude into an X and Y component (with the original X acting as the hypothenuse). This is then converted into the appropriate wheel velocities. This is a critical step, as it allows the robot to move in any direction, and not just along the X axis, while maintaining the capabilities of NAV2.

Also understand that the code running the computation above is housed in a seperate node for easy re-use. This is a staple of this research, as code or infrastructure that cannot be reused or recycled is a waste of resources, and does not actively contribute to the community. You can see the utilization of this node in the graph provided below:

<!-- An image of the nodes -->
![Placeholder](/thesis/assets/placeholder_image.png)

Furthermore, when considering ease of use and understanding of the robot, a typical control through the keyboard arrow keys or WASD felt insufficient. As such, a webapp was created to help visualize the commanded wheel angles and velocities through a visual representation of the robot. It also includes a slider element to command linear and angular velocities. This is a critical step, as it allows for the user to understand the robot's movement in a more intuitive way, and allows for easier debugging of the robot's movement.

<!-- Placeholder image for the UI -->
![Placeholder](/thesis/assets/placeholder_image.png)


### Writing an Odom Node for the Swerve Drive Module

_Predominantly software_



### Writing a PID Controller for the Swerve Drive Module

_Predominantly software_



### Ensuring proper TF frames for the Swerve Drive Module

_Predominantly software_



## Deploying a Webpage to Control the Robot

_Predominantly software_



## Deploying Nav2 and Slam Toolbox

_Predominantly software_



## Integrating the twist_mux to manage multiple sources of velocity commands

_Predominantly software_



## Converting the Nav2 Command to integrate with the swerve drive controller

_Predominantly software_



## Validating the Swerve Drive Robot in Gazebo

_Predominantly software_



## Saving a Map for Later Validation

_Predominantly software_




---


# Results



# Links

https://wiki.ros.org/sw_urdf_exporter (ROS1 URDF Exporter)

https://www2.dmst.aueb.gr/dds/pubs/inbook/beautiful_code/html/Spi07g.html#:~:text=All%20problems%20in%20computer%20science,envisioned%20the%20modern%20personal%20computer. (Butler Lampson Quote)

(MAR3) https://github.com/ros-controls/ros2_controllers

