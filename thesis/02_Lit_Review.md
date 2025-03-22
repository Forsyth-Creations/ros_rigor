<!-- Structure of Lit Review:

- Synthesis 
- Critique
- Contrast
- Connection
- Gaps in Literature that My Research Addresses (My Niche)


* A well-constrained system is an intelligent and useful system

Links and Notes:

# Swerve Drive

(LR1) https://ieeexplore.ieee.org/document/10063871
    - Fuzzy logic control for swerve drive robots
    - I don't understand why they used fuzzy logic. Why not use PID instead, and if you want the motor to move fast or slow than you can just use a threshold that you wouldn't want the motor to exceed.
    - It also doesn't generalize to continuous angular control, as it's focus is on discrete values of the steering angle
    - Rise time of angle set seems really long, I don't understand why it was almost 9 seconds

(LR2) https://ieeexplore.ieee.org/document/10307118
    - Term "Azimuth Motors" used to describe swerve drive motors
    - They talk about the construction of their differential drive in this paper, and how other people put the motor directly on the wheel itself. I could also talk about this
    - They use gears for all their things, whereas mine is a belt system
    - 3D printed hubs
    - Their drive motor is going to get its wiring caught the more the wheel pivots, which is a problem I don't have to deal with
    - Their math was good, I used a great deal of that to inspire mine
    - I largely agree with their architecture. I like the distributed motor drivers, but that is almost a default decision with dealing with BLDC
    - They used a joystick, I used Nav2 and ROS2
    - I use a modulous operator to govern the critical point of change between 6.27 and 0 radians, they use some 'if' statement logic

(LR3) https://ieeexplore.ieee.org/document/10242502
    - Great interpretation of brushless DC motor requimrenets given the robot specifications
    - Also using a slip ring for the wiring, which doesn't make sense. They could've avoided that whole design by using the encoder directly on the motor, and enabling a homing sequence in the event of belt slippage
    - BTN Driver
    - They used PID tuning for their drive and steering motors, smart. Maybe I should also do this. It would be a nice addition/give extra data to the thesis

(LR4) https://ieeexplore.ieee.org/document/9752654
    - Seriously not useful. Too many open-ended statements, honestly just a summary of the state of the art. No real conclusions or anything useful

(LR5) https://ieeexplore.ieee.org/document/9593947
    - Great idea to use straight line tragectories to validate the control system. I should do this too, if time allows
    ( I could talk about tuning the Nav2 params to make the vehicle get there faster)

(LR6) https://ieeexplore.ieee.org/document/10242512
    - Discussion of error when trying to achieve a certain x and y position. However, this error condition is handled within Nav2, where it determines if it needs to re-plan based on the achieved final position
    - It also did planned trajectory path testing, which I could absolutely do

(LR7) https://ieeexplore.ieee.org/document/10698061
    - Drive motor is not in a differential drive format, and will cause wires to get caught in the wheel. I don't have this problem, as my motors are on the outside of the wheel, and the wires are routed through the chassis
    - Their communication schema was interesting with a PS4 controller, I can show my webapp as a counterpoint to this

(LR8) https://ieeexplore.ieee.org/document/6717252
    - A single motor that drives all the pivots by a chain? Yeah, that's not a good idea mechanically. Especially at this scale, without using tensioners
    - Actually, I don't like their design, but they had some great insight into managing battery consumption and power management. I should look into this more

# Navigation in ROS

(LR9) https://ieeexplore.ieee.org/document/9593984
    - This is just a turtlebot. They have an RPLidar (equivalent) on the top of the robot, and a Raspberry Pi Pico. Using Micro-Ros. Using Cartographer (Google I think?).  

(LR10) https://ieeexplore.ieee.org/document/10252030
    - Dynamic actors in the environment with the robot. HuNav agents
    - Slightly out of scope, I think making this work with my setup would cause a large delay. I'd like to talk about the intents of this paper, though, and suggest ways Nav2 might be able to do this. Maybe I can spawn a random box in the environment and have the robot navigate around it dynamically?

(LR11) https://ieeexplore.ieee.org/document/8645984
    - This experimented with changed the sensor set (Lidar or Lidar + RGB-D Camera).
    - Once the wheels are PID tuned, I could play around with max speed and how that impacts the straight line error

(LR12) https://www.swervedrivespecialties.com/

(LR13) https://docs.wpilib.org/en/stable/docs/software/kinematics-and-odometry/swerve-drive-kinematics.html

(LR14) https://www.turtlebot.com/

(LR15) https://nav2.org/

(LR16) https://github.com/SteveMacenski/slam_toolbox

-->

# Literary Review

## Introduction

The literature on swerve drive systems and navigation in ROS2 is limited, as ROS2 was only introduced eight years ago at the time of this research. As a result, the reviewed papers are drawn from the past decade. These papers cover various topics, including fuzzy logic control, motor construction, PID tuning, and the use of different sensors for navigation. The focus of these papers spans mechanical, electrical, and software aspects in their proposed solutions. To keep this review concise, key assertions from these papers will be summarized and critiqued, with comparisons and connections to the research presented in this thesis. This section aims to bridge the gap between the researchers' needs and the objectives of this study. Direct implementations will be mentioned, but will not be discussed directly to avoid confusion between the approaches used within external papers compared to the approach taken in this body of research. Those implementation will be discussed within the *Methods and Results* section of this thesis.

## Motor on drive axis

There are two main approaches to driving the wheels of a swerve drive system: attaching the motor directly to the wheel or using a belt system. The former is common in existing literature, but it can lead to wiring issues and increased complexity in the design. In contrast, the research presented in this paper utilizes a belt system to drive the wheels, which simplifies the design and mitigates potential wiring problems. However, this approach is not without its drawbacks, as it can lead to belt slip and other mechanical issues.

Commercially available swerve drive modules, such as those from Swerve Drive Specialties, utilize a differential drive system, which brings mechanical and computational complexity at the gain of a more compact design. It also alleviates the wiring issues associated with directly attaching the motor to the wheel. This design choice is not without its drawbacks, as increased mechanical complexity can lead to increased failure rates and maintenance requirements. The research will not focus on this design choice, as it is outside the scope of this study. The characteristics of the two different approaches are shown below:

| Design Choice                    | Advantages                               | Disadvantages                                                                                      |
|----------------------------------|------------------------------------------|----------------------------------------------------------------------------------------------------|
| Motor on Drive Axis              | - Reduced mechanical complexity          | - Forbids continuous pivoting due to wire wrap (unless a slip right it used)                       |
|                                  |                                          | - Driving wheel directly on motor without proper consideration can yield a bent shaft              |
|                                  |                                          | - Increased wiring complexity to account for the drive motor                                       |
| Differential for Single Module   | - Reduced wiring complexity              | - Increased mechanical complexity                                                                  |
|                                  | - Allows for continuous pivoting         | - Potential increase in failure rates and maintenance requirements due to increased complexity     |
|                                  |                                          | - Increased computational complexity to account for ratio of motors to wheels                      |

## Motor on radial axis

Both research and professional swerve drive modules agree that added complexity is not needed for the radial axis. Typically, the radial axis moves the primary axial module with either belts, or gears. The research presented in this paper utilizes a belt system to drive the wheels, which simplifies the design and mitigates potential wiring problems. To account for belt slip, the research adds a homing limit switch. In other conditions, you could satisfy this with a gear ratio attached to an absolute encoder, but since absolute encoders are expensive, this is a good compromise. Additionally, integrating this system would increase the physical size of the proposed module, which is not ideal for simplicity.

## Gears vs belt system

The research presented in this paper utilizes a belt system to drive the wheels, which simplifies the design and mitigates potential wiring problems. At hight speeds or under force conditions adverse to the commanded motion, the belt system can slip. This is not a problem with gears, but gears that meet the size requirement of this outdoor swerve drive spec are unavailable or monetarily prohibitive. The research presented in this paper utilizes a belt system to drive the wheels, which simplifies the design and mitigates potential wiring problems.

## Tuning individual motors to move as commanded with PID or with fuzzy logic

There are three schools of thought for tuning the motors of a swerve drive system: PID tuning, fuzzy logic, or reinforcement learning to tune the control based on a variety of environmental factors. The research presented in this paper utilizes PID tuning, as it is the most common and well-understood method for tuning motors in a swerve drive system. Fuzzy logic is less common and is not as well understood, but it can be useful in certain situations. In the body of research, it also has a tendency to operate on a discrete case, whereas we are concerned with continuous motion of the radial axis. Reinforcement learning is an emerging field that has the potential to revolutionize the way we tune motors in a swerve drive system, but it is not yet widely used in practice. A great example of this is the use of Issac Sim to train Blue, a two-legged Star Wars robot to walk. This is outside the scope of this research, but it is an interesting area of study.

## Control Systems (Remote Controllers and ROS2 Options)

The research presented in this paper utilizes a web app to control the robot, which is a novel approach that is not commonly used in the literature. Most research uses a joystick or other physical controller to control the robot. The web app allows for greater flexibility and ease of use and better visualization of the solution set, as it can be accessed from any device with a web browser. This is a significant advantage over traditional controllers, which are often limited to a single device. The web app also allows for greater customization and control over the robot's behavior, as it can be easily modified to suit the user's needs. This is a significant advantage over traditional controllers, which are often limited in their functionality. Additionally, instead of directly controlling the robot, the controller is piped into the control logic through a twist_mux, a multiplexer that uses priority to determine which command to execute. This is a significant advantage over traditional controllers, which often require direct control of the robot. Without this, there would not be a convenient way to switch between human control and autonomous control, without the addition of a mode handler.

## 3D printed parts that bring down cost?

This research leans into the 3D printing of parts to reduce the cost of the robot. This is becoming more common in literature, but does not often appear in conversation when the problem set is software driven. However, those who care about software make their own hardware, and thus it was important to develop a reliable mechanical system that would interface cleanly with the software stack. This is a significant advantage over traditional methods, which often rely on expensive and complex hardware. The 3D printed parts used in this research are not only cost-effective, but they are also lightweight and easy to manufacture. 

## Turtlebot Perception "State of the Art"

"Turtlebot" is a term used to describe a small, low-cost robot that is often used for research and education. Turtlebot4 is the most recent iteration of the work, relying on a perception stack of both a puck lidar and a depth camera. Mirroring the Turtlebot implementation, this research leverages a depth camera. The intent of this robot was to create a framework to scale to any need, and support additional sensors. Moreover, the design presented in the research is an MVP of a swerve drive robot, and is not intended to be a fully-featured robot. The goal is to provide a foundation for future research and development in the field of swerve drive robots, while also highlighting the tuning and creation process of the vehicle. 

## Mathematics Behind the Kinematic Model

The mathematics to control this swerve drive robot are largely inspired by LR2. 