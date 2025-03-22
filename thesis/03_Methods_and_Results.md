# Methods and Results

## Introduction

This section details the methods used to develop the swerve drive robot in ROS2, including the development of the robot model, the implementation of the slam_toolbox and Nav2 stack, and the testing of the robot in simulation. The section is divided into several subsections, each detailing a specific aspect of the development process. Note that all source code and models are available on the GitHub repository for this project; the link is provided in the appendix. As this work is ongoing, attached to the Github link is a specific commit hash. You may use this to check out the code at the time of this writing.

## Development of the Model

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
- The X Axis is the most crucial axis to align: for Nav2, the X axis must be aligned with the forward motion of the robot. This is the default for the URDF exporter, so it is important to ensure that the model is aligned with the X axis in Solidworks.
- For the pivots to function appropriately, the pivot axis must be aligned on their Z axis. Otherwise you risk improper or lopsided movement of the modules

When developing the model, the best approach is the design the model in full, developing submodules where available. Following this, you can proceed with adding points and reference axis. Remember, for the sake of simplified axis, try and keep all your axis aligned with one another. This will make it easier to export to URDF, and easier to debug later. Since this is foundational to the structure of the robot, and changing it will require work, it is best to get this right the first time.


## Adapting the model to be URDF Compliant




## Exporting the model to a URDF, and its Common Structures




## Writing a Launch File to "Spawn" the model in Gazebo




## Writing a World file for the Robot to Navigate in Gazebo




## Writing a ROS Controller Node for Swerve Drive Robotics




### Writing an Odom Node for the Swerve Drive Module




### Writing a PID Controller for the Swerve Drive Module




### Ensuring proper TF frames for the Swerve Drive Module




## Deploying a Webpage to Control the Robot




## Deploying Nav2 and Slam Toolbox




## Integrating the twist_mux to manage multiple sources of velocity commands




## Converting the Nav2 Command to integrate with the swerve drive controller




## Validating the Swerve Drive Robot in Gazebo




## Saving a Map for Later Validation





---


# Results



