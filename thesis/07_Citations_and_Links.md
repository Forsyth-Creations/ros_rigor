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

(LR17) https://github.com/ros-navigation/navigation2/blob/humble/nav2_voxel_grid/README.md

-->


## Github Link

https://github.com/Forsyth-Creations/ros_rigor
Commit hash: --------------------------------

## Used Links

1. ✔
2. ✔ 
3. ✔
4. ✔
5. ✔
6. ✔
7. ✔
8. ✔
9. ✔
10. ✔
11. ✔
12. ✔
13. ✔
14. ✔
15. ✔
16. ✔
17. ✔
18. ✔
19. ✔
20. ✔
21. ✔
22. ✔
23. ✔

## Links

1. LR1: https://ieeexplore.ieee.org/document/10063871
2. LR2: https://ieeexplore.ieee.org/document/10307118
3. LR3: https://ieeexplore.ieee.org/document/10242502
4. LR4: https://ieeexplore.ieee.org/document/9752654
5.  LR5: https://ieeexplore.ieee.org/document/9593947
6. LR6: https://ieeexplore.ieee.org/document/10242512
7.  LR7: https://ieeexplore.ieee.org/document/10698061
8.  LR8: https://ieeexplore.ieee.org/document/6717252
9. LR9: https://ieeexplore.ieee.org/document/9593984
10. LR10: https://ieeexplore.ieee.org/document/10252030
11. LR11: https://ieeexplore.ieee.org/document/8645984
12. LR12: https://www.swervedrivespecialties.com/
13. LR13: https://docs.wpilib.org/en/stable/docs/software/kinematics-and-odometry/swerve-drive-kinematics.html
14. LR14: https://www.turtlebot.com/
15. LR15: https://nav2.org/
16. LR16: https://github.com/SteveMacenski/slam_toolbox
17. LR17: https://github.com/ros-navigation/navigation2/blob/humble/nav2_voxel_grid/README.md
18. R1: https://wiki.ros.org/sw_urdf_exporter (ROS1 URDF Exporter)
19. R2: https://www2.dmst.aueb.gr/dds/pubs/inbook/beautiful_code/html/Spi07g.html#:~:text=All%20problems%20in%20computer%20science,envisioned%20the%20modern%20personal%20computer. (Butler Lampson Quote)
20. R3: https://github.com/ros-controls/ros2_controllers
21. R4: https://ieeexplore.ieee.org/document/6147128
22. https://www.outlookbusiness.com/start-up/news/meet-blue-the-cute-little-ai-robot-built-by-nvidia-disney-and-google
23. https://www.sciencedirect.com/science/article/pii/S0263224119302489?casa_token=6tl98m6DxOQAAAAA:HaJ0SL0x3tIQ6LKtYFuq_PTn2jUMhKIraj6jPxuWhBSjUVvQ-S2s9FfCJhsnK6Xoxz9aRbJn
24. https://www.nvidia.com/en-us/autonomous-machines/embedded-systems/jetson-orin/

## Equivalent Citations

**When the paper is done, I will add the equivalent citations here. For now, the links are enough.**