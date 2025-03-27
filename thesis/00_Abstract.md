# Title

Precision Robotics with Lightweight Hardware and Software: A Swerve Drive Implementation

Henry Forsyth

# Keywords

Robotics, Swerve Drive, Intel RealSense D435i, slam_toolbox, Nav2, PID tuning, Gazebo, twist_mux, ROS2, simulation, path planning, cost map, pose, quaternion, navigation stack, depth camera, four-wheel drive, omnidirectional movement, software stack, hardware architecture, general purpose robotics, computational power, optimization, real-world application, robotic design.

# Specific Abstract

The era of AI demands ever increasing computational power. As we dawn the era of general purpose robotics, this trend continues. For complex robotics systems, thoughtful choices of both software and hardware architecture are required for optimal performance. This paper explores one such implantation: a swerve drive robot, leveraging an Intel RealSense D435i depth camera, to navigate a room using the slam_toolbox and Nav2 stack. Using Gazebo as the simulation environment, and with the eventual goal of moving the software stack to a physical robot, this paper explores the optimal development choices for robotic design. It also explores considerations for compute utilization to prevent system bottlenecks.

# General Audience Abstract

This paper discusses the development and optimization of a four-wheel robotic system. It highlights the importance of selecting appropriate software and hardware for achieving optimal performance. The study involves a robot equipped with a depth camera for navigation, utilizing specific software tools for mapping and control. The system is tested in various environments, both in simulation and with the aim of real-world application.

# Dedications

Thidapat Chantem
Ryan Williams
Scot Ransbottom

---

Catherine Hebert
David Spadaccia
Michael Yanoshak
My Family

# Acknowledgements

I would like to thank my committee members, Dr. Thidapat Chantem, Dr. Ryan Williams, and Dr. Scot Ransbottom, for their guidance and support throughout this project. I would also like to thank David Spadaccia, Michael Yanoshak, and Catherine Hebert for encouraging me through this process. Finally, I would like to thank my family for their support.

# Key Terms

- "Stack" or "Software Stack": A collection of software tools that work together to achieve a common goal.
- PID Tuning: A method of adjusting the parameters of a proportional-integral-derivative controller to achieve optimal performance.
- Gazebo: A robot simulation environment that allows for testing of robotic systems in a virtual environment.
- slam_toolbox: A set of tools for performing simultaneous localization and mapping (SLAM) in robotic systems.
- Nav2: A navigation stack for ROS2 that provides tools for robot navigation.
- Swerve Drive: A type of robotic drive system that allows for omnidirectional movement.
- Cost Map: A map of the environment that assigns a cost to each cell based on its traceability. Obstacles are assigned a higher cost, while open areas are assigned a lower cost. Used for path planning.
- Pose: The position and orientation of a robot in space. This is typically represented as a 3D coordinate and a quaternion.
- Quaternion: A mathematical construct used to represent rotations in 3D space. It is a more compact representation than Euler angles. Instead of using a six number representation, it uses four numbers to represent the same information. This approach is more efficient for computation and avoids issues with gimbal lock.
- Swerve Module: The individual wheel assembly of a swerve drive robot. Each module can pivot independently to allow for omnidirectional movement.
- Swerve Drive Radial Axis: The axis around which the swerve module pivots. This is typically perpendicular to the ground and parallel to the wheel. Used synonymously with "pivot axis".
- Swerve Drive Axial Axis: The axis along which the wheel rotates. This is typically parallel to the ground and perpendicular to the swerve module. Used synonymously with "drive axis".
- Twist_mux: A tool used to manage multiple sources of velocity commands in ROS2.
- "Differential Drivetrain" vs "Swerve Drive Differential": A differential drivetrain is a type of drive system that uses two wheels to control the direction of the robot. A swerve drive differential describes the differential drive system of a swerve drive robot. This entails two motors on a module, which depending on the combination of commands to the two motors and their difference in motion can yield either a radial, axial, or radial and axial motion. This is a unique feature of swerve drive robots, and is not present in traditional differential drive systems.
- MVP or Minimum Viable Product. The simplest version of a product that can be released to the market.
- Puck Lidar: A type of Lidar sensor that is used for navigation and mapping in robotic systems. It is a compact and lightweight sensor that is ideal for use in small robots. It's field of view is 360 degrees, with most having a range of 10-20 meters. It is typically used for indoor navigation and mapping.
- IMU or Inertial Measurement Unit. A sensor that measures the orientation, velocity, and gravitational forces acting on an object. It typically consists of an accelerometer, gyroscope, and magnetometer. It is used for navigation and control in robotic systems, usually alongside other sensors to prevent IMU drift (error accumulation over time due to the antiderivative of sensor data).
- SLAM: Simultaneous Localization and Mapping. A technique used in robotics to create a map of an unknown environment while simultaneously localizing the robot within that environment. Aids in generating a proper transform between the robot and the world frame (typically the base_link and map frame in ROS)
