# Title

Precision Robotics with Lightweight Hardware and Software: A Swerve Drive Implementation

Henry Forsyth 

# Keywords

Robotics, Swerve Drive, Intel Realsense D435i, slam_toolbox, Nav2, PID tuning, Gazebo

# Specific Abstract

The era of AI demands ever increasing computational power. As we dawn the era of general purpose robotics, this trend repeats. For complex robotics systems, thoughtful choices of both software and hardware architecure are required for optimal performance. This paper explores one such implentation: a swerve drive robot, leveraging an Intel Realsense D435i depth camera, to navigate a room using the slam_toolbox and Nav2 stack. The controller is optimized using PID tuning and the robot is tested in a variety of environments. Using Gazebo as the simulation environment, and with the eventual goal of moving the software stack to a physical robot, this paper explores the optimal development choices for robotic design.

# General Audience Abstract

This paper discusses the development and optimization of a four-wheel robotic system. It highlights the importance of selecting appropriate software and hardware for achieving optimal performance. The study involves a robot equipped with a depth camera for navigation, utilizing specific software tools for mapping and control. The system is tested in various environments, both in simulation and with the aim of real-world application.

# Dedications

Thidapat Chantem
Ryan Williams
Scot Ransbottom

---

David Spadaccia
Michael Yanoshak
Catherine Hebert
My Family

# Acknowledgements

I would like to thank my committee members, Dr. Thidapat Chantem, Dr. Ryan Williams, and Dr. Scot Ransbottom, for their guidance and support throughout this project. I would also like to thank David Spadaccia, Michael Yanoshak, and Catherine Hebert for their assistance with the project. Finally, I would like to thank my family for their unwavering support.

# Key Terms

- "Stack" or "Software Stack": A collection of software tools that work together to achieve a common goal.
- PID Tuning: A method of adjusting the parameters of a proportional-integral-derivative controller to achieve optimal performance.
- Gazebo: A robot simulation environment that allows for testing of robotic systems in a virtual environment.
- slam_toolbox: A set of tools for performing simultaneous localization and mapping (SLAM) in robotic systems.
- Nav2: A navigation stack for ROS2 that provides tools for robot navigation.
- Swerve Drive: A type of robotic drive system that allows for omnidirectional movement.
- Cost Map: A map of the environment that assigns a cost to each cell based on its traversability. Obstables are assigned a higher cost, while open areas are assigned a lower cost. Used for path planning.
- Pose: The position and orientation of a robot in space. This is typically represented as a 3D coordinate and a quaternion.
- Quaternion: A mathematical construct used to represent rotations in 3D space. It is a more compact representation than Euler angles.
