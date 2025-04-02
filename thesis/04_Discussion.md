# Discussion of Results

## Discussion of Swerve Drive Module

A great deal of the mechanical work behind this research was intended to present a counter argument, or a counter solution, to the accepted slip ring design seen throughout the body of existing work. One key benefit of the slip ring system is ground-truth encoder data, where you don't need to back calculate the angle of the wheel nor do you need to worry about belt slipage. However, the slip ring system is not without its drawbacks. The slip ring system is expensive, and it is a single point of failure. The swerve drive module presented in this research is a more robust solution to the problem of swerve drive. It is also a more cost effective solution, as it does not require the use of an expensive slip ring.

## Descritized PID Tuning vs Zeiger-Nichols Method

The discretized PID tuning method was used to tune the PID controller for the swerve drive module. This method is a more modern approach to PID tuning, and it is based on the idea that the system can be modeled as a discrete-time system. The Zeiger-Nichols method is a more traditional approach to PID tuning, and it is based on the idea that the system can be modeled as a continuous-time system. The discretized PID tuning method was found to be more effective than the Zeiger-Nichols method for this application, as you can force its P term to encourage a faster convergence. The Zeiger-Nichols method is more suited for systems that are already well tuned, and do not require this level of tuning. Below are the results of the PID and Ziegler-Nichols tuning methods, along with their resulting values for a 0 to 2 m/s jump in speed:


| **Tuning Method**    | **Kp** | **Ki** | **Kd** | 
|----------------------|--------|--------|--------|
| **Discretized PID**  | 1.481  | 0      | 0.311  |
| **Ziegler-Nichols**  | 0.1304 | 0.157  | 1.587  |

**Legend:**
- **Kp**: Proportional gain
- **Ki**: Integral gain
- **Kd**: Derivative gain

## Raspberry Pi 5 Utilization and Performance

Selecting the Raspberry Pi 5 came with known compute limitations. In reality, for a task like this, the NVIDIA Jetson would be the ideal choice with its GPU capabilities. However, the Raspberry Pi 5 was selected for its low cost and ease of use. However, with GPU capabilities, we could branch into more accurate 3D SLAM algorithms [23], which would allow for more accurate localization and mapping. While this was never explicitly tested, an assumption was made that on such a light compute, the sensor set would need to be limited to maintain CPU utilization within a reasonable range. As such, only one sensor (the depth camera) was used for localization and mapping, whereas most literature [9] also leverages a puck lidar to better keep track of the robot's position.  

## Recommendations for Future Work With Issac Sim and Nvidia Jetson

The researcher expects that in just a few years, simulation environments along with their robotic models will move to Issac Sim, where you can run thousands of test runs and generate a custom reinforcement learning model based on the environments the robot has been trained in. Disney and NVIDIA have already made great strides toward this with their Blue robot, which runs two NVIDIA computes (presumably Jetson Orins [24]) within its chassis. This will not only allow for more accurate simulations, but faster online learning and the ability to quickly adapt to new environments. This Gazebo and ROS2 research acts as the gateway to understanding of the fundementals of standard training and controller methods as we approach the next generation of robotics. The researcher recommends that future work focus on the integration of Issac Sim with the NVIDIA Jetson platform, as this will allow for more accurate simulations and faster online learning. This will also allow for the development of more complex algorithms, such as reinforcement learning, which will be necessary for the next generation of robotics.


