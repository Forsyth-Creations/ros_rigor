# Chapter 1: Introduction

Swerve drive robots are a unique hybrid of traditional four-wheel drive and mecanum drive systems. Pivoting radially, they offer the ability to strafe and the ability to turn in place. These robotic systems are commonplace within the First Robotics Competition (FRC) community, where they are used to navigate complex obstacle courses. This work has, however, not been made readily available to the ROS2 community, as these types of robots are rare in the research community. This paper aims to bridge that gap by providing a detailed overview of the development and optimization of a swerve drive robot within the ROS2 ecosystem. The research also details optimization of perception methods for lightweight hardware, specifically as it pertains to navigation and mapping. The goal is to provide a roadmap for researchers and developers who are interested in building and optimizing swerve drive robots using ROS2.

## Section 1.1: Background

### Swerve Drive

---

![Diagram of My Swerve Module](../thesis/assets/placeholder_image_2.png "Placeholder Image")

Figure 1: Diagram of Swerve Module

---

![Diagram of Existing Swerve Module](../thesis/assets/placeholder_image_2.png "Placeholder Image")

Figure 2: Diagram of Existing Swerve Module

---

Swerve drive robots are unique in that they have four independent wheel modules that can pivot radially. This allows the robot to move in any direction without changing the orientation of the robot itself, ideal for precision and moving of heavy of awkwardly sized payloads. The modules are typically driven by a combination of motors and encoders, which allow for precise control of the robot's movement. The implementation shown above diverges from traditional swerve drive systems, to account for the larger wheel size. This permits it to navigate more effectively in rough terrain.

### ROS2

ROS2 is a flexible framework for writing robot software. It is a collection of tools, libraries, and conventions that aim to simplify the task of creating complex and robust robot behavior across a variety of robotic platforms. One misconception is that ROS2 is real-time. It is not but provides the flexibility for developers to implement real-time systems. The foundations for this project relies on two models of the ROS2 ecosystem: the Publisher-Subscriber model and the Client-Server model. The former is used for communication between nodes, while the latter is used for within parts of the Nav2 stack. This study presents several diagrams of the ROS2 architecture, to provide a clear understanding of the system. For this research, it is critical to understand the decision to use ROS2. More than just being the state of the art, its communication protocol (DDS) is more fault tolerant than the ROS (ROS1) protocol, which leveraged UDP and TCP. On the surface there is no problem with applying these protocols, but a single ROS master node to coordinate all the nodes in the system is a single point of failure. This is not the case with ROS2, which is a distributed system. This will be discussed briefly within the context of the paper, but the reader should be aware of this distinction. Note that, when a distinction needs to be made between ROS1 (ROS) and ROS2, this paper favors explicitly calling it "ROS1" to provide clarity, despite this not being the standard convention. Using the term "ROS" will indicate a generalization of the two, where either can apply.

### slam_toolbox

The slam_toolbox is a set of tools for performing simultaneous localization and mapping (SLAM) in robotic systems. It is a collection of ROS2 packages that help generate a 2D map of the robot's environment and localize the robot within that map. The slam_toolbox is used in this study to generate a map of the robot's environment and localize the robot within that map. The slam_toolbox is a key component of the Nav2 stack, which is used for robot navigation. In terms of this study, the slam_toolbox exclusively focuses on mapping, which the Nav2 stack leverages to generate a "cost map" of the environment, which is used for path planning.

### Nav2

Nav2 is a software stack for ROS2 that provides tools for robot navigation. It is a collection of ROS2 packages that help the robot navigate its environment by generating a "cost map" of the environment and planning a path through that map. The "cost map" can be thought of as a gradient, where obstacles take on a darker hue, with higher costs. To stick with this mental image, that means our traversal will remain in the lighter, lower-cost troughs of the cost map. Nav2 is supported by the Open Navigation Stack (OpenNav) project, which aims to provide a flexible and extensible navigation stack for ROS2. In the case of this study, we use the "Waypoint" modality, where the robot localizes and estimates its starting pose and is then commanded to achieve a different pose.

### Gazebo and Rviz

Gazebo is a robot simulation environment and physics engine that acts as a "stand in" for the robot and its environment. It is used to test robotic systems in a virtual environment before deploying them on a physical robot. Rviz is a 3D visualization tool for ROS that allows you to visualize the robot's environment and sensor data in real-time. Both tools are used to test the robot's navigation and control algorithms in a simulated environment before deploying them on a physical robot.

## Section 1.2: Problem Statement

The problem this paper aims to address is the lack of information on the development and optimization of swerve drive robots within the ROS2 ecosystem. While swerve drive robots are common in the FRC community, they are rare in the research community. This paper aims to bridge that gap by providing a detailed overview of the development and optimization of this ROS2 robot, providing a roadmap for researchers and developers who are interested in building and optimizing swerve drive robots using ROS2. 

## Section 1.3: Research Objectives

The research objectives of this paper are as follows:

- Develop a Swerve Drive robot in Solidworks, accounting for proper frames of reference for the kinematic model of the robot
- Export the URDF model to Gazebo and Rviz for simulation
- Implement the slam_toolbox and Nav2 stack for mapping and navigation
- Test the robot in a variety of environments within simulation, and prepare the software architecture for deployment on a physical robot

## Section 1.4: Research Questions

The research questions this paper aims to answer are as follows:

- What existing toolchains can be leveraged to develop a swerve drive robot in ROS2?
- Are there common approaches to navigation within the ROS2 ecosystem that can be applied to swerve drive robots?
- What perception systems are best suited for swerve drive robots? What are the trade-offs between different perception systems?
- Where does current research fall short explaining the nuances of swerve drive robots?  _This is a question I'm considering not including in the final paper._

## Section 1.5: Scope and Limitations

The scope of this paper is on traditional methods of robot development as they relate to ROS2. While this paper will branch into leveraging machine learning as a means of optimizing the robot's PID parameters, as well as discussions of A* search patterns for navigation, softwares like Issac Lab or Omniverse are not within the scope of this paper. However, these are areas of interest of the author, and will be explored in future works. 


