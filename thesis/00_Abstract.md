# Title

Precision Robotics with Lightweight Hardware and Software: A Swerve Drive Implementation

Henry Forsyth

# Keywords

Robotics, Swerve Drive, Intel RealSense D435i, slam_toolbox, Nav2, PID tuning, Gazebo, twist_mux, ROS2, simulation, path planning, cost map, pose, quaternion, navigation stack, depth camera, four-wheel drive, omnidirectional movement, software stack, hardware architecture, general purpose robotics, computational power, optimization, real-world application, robotic design.

# Specific Abstract

The era of AI demands ever increasing computational power. As we dawn the era of general purpose robotics, this trend continues. For complex robotics systems, thoughtful choices of both software and hardware architecture are required for maximizing perforamce: how do we deploy the most complex software systems on the most minimal hardware all while maximizing usability and reducing error? This question impacts the saftey of future systems, as robotic error can mean anything on the order of property damage to human endangerment. This paper explores an advanced robotic system with lightweight compute hardare: a swerve drive robot, leveraging an Intel RealSense D435i depth camera, to navigate a room using the slam_toolbox and Nav2 stack. This robot uses a Raspberry Pi 5 as its main compute. A swerve drive is a unique hybrid of traditional four-wheel drive and mecanum drive systems. Pivoting radially, they offer the ability to strafe and the ability to turn in place. This design, built with eight motors, is intended for factories and industry where moving high-value, high weight products in a controlled manor is critical. Using Gazebo as the simulation environment, and with the eventual goal of moving the software stack to a physical robot, this paper explores the development pipeline for robot, and how to leverage computationally light algorithms for a high-yield product. It also explores considerations for compute utilization to prevent system bottlenecks. The overall goal is to provide a roadmap for researchers and developers who are interested in building and optimizing swerve drive robots using ROS2 on less expensive hardware, specifically the Raspberry Pi 5.

# General Audience Abstract

This paper discusses the development and optimization of a four-wheel robotic system. It highlights the importance of selecting appropriate software and hardware for achieving high software complexity and low error on a low power compute. The study involves a robot equipped with a depth camera for navigation, utilizing specific software tools for mapping and control. The system is tested in various environments, both in simulation and with the aim of real-world application.

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

- "Stack", "Software Stack", "Software Suite": A collection of software tools that work together to achieve a common goal. Typically used with more nuiance, but for the purpose of this paper they will mean essentially the same.
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
- TF: The ROS ecosystem transformation tree topic which allows physical bodies to register their position based on a parent transformation frame. TF2 is its successor, but is not used within the body of this work. 
- URDF: Universal Robot Description Format. A file format used to describe the physical properties of a robot, including its geometry, kinematics, and dynamics. It is used in ROS to represent the robot model and is typically used in conjunction with Gazebo for simulation.
- SDF or Simulation Description Format. A file format used to describe the physical properties of a robot and its environment. It is used in Gazebo for simulation and is typically used in conjunction with URDF for representing the robot model.
- ROS2: Robot Operating System 2. An open-source framework for building robotic systems. It provides a set of tools and libraries for developing robot software and is widely used in the robotics community.
- Bench Test: A test performed on a robot or robotic system to evaluate its performance and functionality. A "Bench Test" usually happens on a "Test Bench", which is a physical or virtual environment where the robot can be tested and evaluated. This is typically done before deploying the robot in a real-world environment. A "Bench" is best if it parallels the hardware and software of the final product, but in the case where this is not possible the "Bench Tests" that are run should add extra headroom and considerations for the final hardware and software stack (i.e. if your bench test on a Raspberry Pi 5 is run with limited hardware, and the company requirement for CPU utilization is 60%, then the bench test should be run at 40% CPU utilization to allow for headroom. Note this proposed test is a rough estimate, and is mainly meant to highlight the considerations in development). 
