

# Literary Review

## Introduction

The literature on swerve drive systems and navigation in ROS2 is limited, as ROS2 was only introduced eight years ago at the time of this research. As a result, the reviewed papers are drawn from the past decade. These papers cover various topics, including fuzzy logic control, motor construction, PID tuning, and the use of different sensors for navigation. The focus of these papers spans mechanical, electrical, and software aspects specific to their proposed solutions. To keep this review concise, key assertions from these papers will be summarized and critiqued, with comparisons and connections being drawn to the research presented in this thesis. This section aims to bridge the gap between the researchers' needs and the objectives of this study. Direct implementations done within the body of this workd will be mentioned, but will not be discussed directly to avoid confusion. Those implementation will be discussed within the *Results* section of this thesis.

## Motor on drive axis

There are two main approaches to driving the wheels of a swerve drive system: attaching the motor directly to the wheel or using a belt system [2][3][4][5][6][7]. The former is common in existing literature, but it can lead to wiring issues and increased complexity in the design. In contrast, the research presented in this paper utilizes a belt and pulley system to drive the wheels, which simplifies the design and mitigates potential wiring problems. However, this approach is not without its drawbacks, as it can lead to belt slip and other mechanical issues. Commercially available swerve drive modules, such as those from Swerve Drive Specialties [12], utilize a differential drive system too. The characteristics of the two different approaches are shown below:

| Design Choice                    | Advantages                               | Disadvantages                                                                                      |
|----------------------------------|------------------------------------------|----------------------------------------------------------------------------------------------------|
| Motor on Drive Axis              | - Reduced mechanical complexity          | - Forbids continuous pivoting due to wire wrap (unless a slip right it used)                       |
|                                  |                                          | - Driving wheel directly on motor without proper consideration can yield a bent shaft              |
|                                  |                                          | - Increased wiring complexity to account for the drive motor                                       |
| Swerve Differential              | - Reduced wiring complexity              | - Increased mechanical complexity                                                                  |
|                                  | - Allows for continuous pivoting         | - Potential increase in failure rates and maintenance requirements due to increased complexity     |
|                                  |                                          | - Increased computational complexity to account for ratio of motors to wheels                      |

## Motor on radial axis

Both research and professional swerve drive modules agree that added complexity is not needed for the radial axis. Typically, the radial axis moves the primary axial module with either belts, or gears. The research presented in this paper utilizes a belt system to drive the wheels, which simplifies the design and mitigates potential wiring problems. To account for belt slip, the research adds a homing limit switch. In other conditions, you could satisfy this with a gear ratio attached to an absolute encoder, but since absolute encoders are expensive, this is a good compromise. Additionally, integrating this system would increase the physical size of the proposed module, which is not ideal for simplicity. There are designs that chain the radial wheels together but this contravenes the idea of a swerve drive system and limits mobility [8].

## Gears vs belt system

The research presented in this paper utilizes a belt system to drive the wheels, which simplifies the design and mitigates potential wiring problems. At hight speeds or under force conditions adverse to the commanded motion, the belt system can slip. This is not a problem with gears, but gears that meet the size requirement of this outdoor swerve drive spec are unavailable or monetarily prohibitive. The research presented in this paper utilizes a belt system to drive the wheels, which simplifies the design.

## Tuning individual motors to move as commanded with PID or with fuzzy logic

There are three schools of thought for tuning the motors of a swerve drive system: PID tuning, fuzzy logic, or reinforcement learning to tune the control based on a variety of environmental factors. The research presented in this paper utilizes PID tuning, as it is the most common and well-understood method for tuning motors in a swerve drive system. Fuzzy logic is less common, revolving around discrete controls given an input speed [1]. However, this usecase can be covered through PID tuning, and even better through a discretized PID tuning algorithm (shown later). Reinforcement learning is an emerging field that has the potential to revolutionize the way we tune motors in a swerve drive system, and is quickly becoming common practice. A great example of this is the use of Issac Sim to train Blue [22], a two-legged Star Wars robot to walk. This is outside the scope of this research, but it is an interesting area of study.

## Control Systems (Remote Controllers and ROS2 Options)

The research presented in this paper utilizes a web app to control the robot, which is a novel approach that is not commonly used in the literature. Most research uses a joystick or other physical controller to control the robot [7]. The web app allows for greater flexibility and ease of use and better visualization of the solution set, as it can be accessed from any device with a web browser. This is a significant advantage over traditional controllers, which are often limited to a single device. The web app also allows for greater customization and control over the robot's behavior, as it can be easily modified to suit the user's needs, making is scalable. Additionally, instead of directly controlling the robot, the controller is piped into the control logic through a twist_mux, a multiplexer that uses priority to determine which command to execute. Without this, there would not be a convenient way to switch between human control and autonomous control. The web GUI also gives us a quick and easy way to visualize the robot's state and mission critical data, which is not commonly found in the literature apart from using RVIZ.

## 3D PRINTING FOR CHASSIS

This research leans into the 3D printing of parts to reduce the cost of the robot. This is becoming more common in literature, but does not often appear in conversation when the problem set is software driven. However, those who care about software make their own hardware, and thus it was important to develop a reliable mechanical system that would interface cleanly with the software stack. This is a significant advantage over traditional methods, which often rely on expensive and complex hardware. The 3D printed parts used in this research are not only cost-effective, but they are also lightweight and easy to manufacture. 

## Turtlebot Perception "State of the Art"

"Turtlebot" [14] is a term used to describe a small, low-cost robot that is often used for research and education. Turtlebot4 is the most recent iteration of the work, relying on a perception stack of both a puck lidar and a depth camera. Mirroring the Turtlebot implementation, this research leverages a depth camera. The intent of this robot was to create a framework to scale to any need, and support additional sensors. Moreover, the design presented in the research is an MVP of a swerve drive robot, and is not intended to be a fully-featured robot. The goal is to provide a foundation for future research and development in the field of swerve drive robots, while also highlighting the tuning and creation process of the vehicle.

# Slam Toolbox vs Voxel Mapping

While the Nav2 stack supports voxel grids out of the box [17], the presumed additional processing used to determine and maintain this data with online learning was contrary to the hardware selection and goal of the research, which is intended to be minimal (Raspberry Pi 5). Instead, we relied on a basic perception model which we could tune for the environment to capture the map similar to [9][11], passing it to SLAM toolbox [16], and then supply it to Nav2[15]