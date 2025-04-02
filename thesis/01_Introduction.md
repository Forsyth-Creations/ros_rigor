# Chapter 1: Introduction

Swerve drive robots are a unique hybrid of traditional four-wheel drive and mecanum drive systems. With wheels that can pivot radially, they offer the robot the ability to strafe and to turn in place. These robotic systems are commonplace within the First Robotics Competition (FRC) community [13], where they are used to navigate complex obstacle courses. This work has, however, not been a large interest of the ROS2 community, as these types of robots are rare in research. This paper aims to bridge that gap by providing a detailed overview of the development and optimization of a swerve drive robot within the ROS2 ecosystem. The research also details optimization of perception methods for lightweight compute hardware, specifically as it pertains to navigation and mapping. The goal is to provide a roadmap for researchers and developers who are interested in building and optimizing swerve drive robots using ROS2 on less expensive hardware, specifically the Raspberry Pi 5.

Robots are becoming increasingly important in a variety of fields, including manufacturing, healthcare, and transportation. As robots become more advanced, they are able to perform more complex tasks and operate in more challenging environments. One area of research that has gained significant attention in recent years is the development of autonomous robots that can navigate and map their environment. This is particularly important for applications such as search and rescue, where robots need to be able to navigate through unknown and potentially hazardous environments. It is also critical for factories, where supplies need to be moved efficiently and accurately. The development of swerve drive robots is an important step towards achieving this goal, as they offer a unique combination of mobility and maneuverability that is well-suited for navigating complex environments. While they come with their own set of challenges, namely their energy consumption and mechanical complexity, they are a promising solution for a variety of applications. 

Current research in this field has used highly simplified and, as a consequence, limiting robotic designs for swerve drive robots. Many rely on slip rings, or motors mounted directly onto the wheels. While there are examples such as NASA's swerve drive car that demand motors connected directly to the wheels for added power, there are simpler design decisions that can be made mechanically to illeviate the electrical complexities brought on by such design choices. Instead, we can use a swerve differential drive module, where two pulleys work in tandem to both drive the wheel, and drive the rotation of the wheel. This design decision is not only simpler, but also more space efficient, as it allows for a more compact design and reduces the number of moving electrical components.  

The introduction of SLAM algorithms, working in conjuction with NAV2, also presents a unique opportunity to marry the best hardware and software solutions to create a robot that can navigate and map its environment in real-time. Deployed to a Raspberry Pi 5, the goal is to deploy this sophisticated software stack on lightweight hardware, keeping a close eye on the frequency and hardware demands of the algorithms. The research will also strive to keep track of robot error, measuing offset for its designed goal and overshoot of the modules to optimize the PID parameters for robotic efficiency. This paper aims to provide a comprehensive overview of the development and optimization of swerve drive robots within the ROS2 ecosystem, with a focus on the challenges and opportunities that arise in this field, and to provide a roadmap for researchers and developers who are interested in building and optimizing their own robots in ROS2 and Gazebo.

The organization of this paper is as follows:

- Chapter 1: Introduction
    - Section 1.1: Problem Statement
    - Section 1.2: Research Questions
    - Section 1.3: Research Objectives
    - Section 1.4: Scope and Limitations
    - Section 1.5: Preliminaries
- Chapter 2: Literature Review
- Chapter 3: Methodology
- Chapter 4: Results
- Chapter 5: Discussion
- Chapter 6: Conclusion
- Chapter 7: Future Work
- Chapter 8: References
- Chapter 9: Appendices

## Section 1.1: Problem Statement

The problem this paper aims to address is the lack of information on the development and optimization of swerve drive robots within the ROS2 ecosystem. While swerve drive robots are common in the FRC community, they are rare in the research community. This paper aims to bridge that gap by providing a detailed overview of the development and optimization of this ROS2 robot, providing a roadmap for researchers and developers who are interested in building and optimizing swerve drive robots using ROS2. 

## Section 1.2: Research Questions

The research questions this paper aims to answer are as follows:

- How can we use ROS2 and Gazebo to simulate and validate our swerve drive robot?
- How can we configure Nav2 for the most computational headroom without increasing error?
- What perception systems are best suited for swerve drive robots? What are the trade-offs between different perception systems?
- If its measurable its manageable. What are key performance metrics for swerve drive robots?

## Section 1.3: Research Objectives

The research objectives of this paper are as follows:

- Develop a Swerve Drive robot in Solidworks, accounting for proper frames of reference for the kinematic model of the robot
- Export the URDF model to Gazebo and Rviz for simulation
- Implement the slam_toolbox and Nav2 stack for mapping and navigation
- Test the robot in a variety of environments within simulation, and prepare the software architecture for deployment on a physical robot
- Configure the software stack for the Raspberry Pi 5, ensuring CPU usage is kept below 70% and RAM usage is kept below 70% under normal operating conditions 

## Section 1.4: Scope and Limitations

The scope of this paper is on traditional methods of robot development as they relate to ROS2. While this paper will branch into leveraging machine learning as a means of optimizing the robot's PID parameters, as well as discussions of A* search patterns for navigation, softwares like Issac Lab or Omniverse are not within the scope of this paper. However, these are areas of interest of the author, and will be explored in future works. 

## Section 1.5: Preliminaries 

### ROS2

ROS2 is a flexible framework for writing robot software. It is a collection of tools, libraries, and conventions that aim to simplify the task of creating complex and robust robot behavior across a variety of robotic platforms. One misconception is that ROS2 is real-time. It is not but provides the flexibility for developers to implement real-time systems. The foundations for this project relies on two models of the ROS2 ecosystem: the Publisher-Subscriber model and the Client-Server model. The former is used for communication between nodes, while the latter is used for within parts of the Nav2 stack. This study presents several diagrams of the ROS2 architecture, to provide a clear understanding of the system. For this research, it is critical to understand the decision to use ROS2. More than just being the state of the art, its communication protocol (DDS) is more fault tolerant than the ROS (ROS1) protocol, which leveraged UDP and TCP. On the surface there is no problem with applying these protocols, but a single ROS master node to coordinate all the nodes in the system is a single point of failure. This is not the case with ROS2, which is a distributed system. This will be discussed briefly within the context of the paper, but the reader should be aware of this distinction. Note that, when a distinction needs to be made between ROS1 (ROS) and ROS2, this paper favors explicitly calling it "ROS1" to provide clarity, despite this not being the standard convention. Using the term "ROS" will indicate a generalization of the two, where either can apply.

### slam_toolbox

The slam_toolbox is a set of tools for performing simultaneous localization and mapping (SLAM) in robotic systems. It is a collection of ROS2 packages that help generate a 2D map of the robot's environment and localize the robot within that map. The slam_toolbox is used in this study to generate a map of the robot's environment and localize the robot within that map. The slam_toolbox is a key component of the Nav2 stack, which is used for robot navigation. In terms of this study, the slam_toolbox exclusively focuses on mapping, which the Nav2 stack leverages to generate a "cost map" of the environment, which is used for path planning.

### Nav2

Nav2 is a software stack for ROS2 that provides tools for robot navigation. It is a collection of ROS2 packages that help the robot navigate its environment by generating a "cost map" of the environment and planning a path through that map. The "cost map" can be thought of as a gradient, where obstacles take on a darker hue, with higher costs. To stick with this mental image, that means our traversal will remain in the lighter, lower-cost troughs of the cost map. Nav2 is supported by the Open Navigation Stack (OpenNav) project, which aims to provide a flexible and extensible navigation stack for ROS2. In the case of this study, we use the "Waypoint" modality, where the robot localizes and estimates its starting pose and is then commanded to achieve a different pose.

### Gazebo and Rviz

Gazebo is a robot simulation environment and physics engine that acts as a "stand in" for the robot and its environment. It is used to test robotic systems in a virtual environment before deploying them on a physical robot. Rviz is a 3D visualization tool for ROS that allows you to visualize the robot's environment and sensor data in real-time. Both tools are used to test the robot's navigation and control algorithms in a simulated environment before deploying them on a physical robot.
