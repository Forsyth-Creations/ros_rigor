from math import atan2, cos, pi, sin
from typing import Tuple


class ModuleKinematics:
    def __init__(self, name, wheel_size, module_position: dict, starting_angle = 0):
        self.name = name
        self.wheel_size = wheel_size
        self.module_position: dict = module_position
        self.angle = starting_angle

    @classmethod
    def normalize_angle(self, angle):
        return ((angle % (2 * pi)) + 2 * pi) % (2 * pi)
    
    @classmethod
    def shortest_angle(cls, current_angle, target_angle):
        sub1 = (target_angle - current_angle + pi)
        sub2 = sub1 % (2 * pi)
        diff = (sub2) - pi
        return current_angle + diff
    
    @classmethod
    def compute_wheel_speed(cls, Vx, Vy, omega, Xi, Yi): # These are the same as the webapp
        return (( (Vx - omega * Yi) ** 2 + (Vy + omega * Xi) ** 2 ) ** 0.5) 

    @classmethod
    def compute_wheel_angle(cls, Vx, Vy, omega, Xi, Yi): # These are the same as the webapp
        """
        Compute the angle of a wheel in a swerve drive system.

        This method calculates the angle of a wheel based on the given velocities and 
        the position of the wheel relative to the robot's center.

        Args:
            Vx (float): The velocity of the robot along the x-axis.
            Vy (float): The velocity of the robot along the y-axis.
            omega (float): The angular velocity of the robot.
            Xi (float): The x-coordinate of the wheel relative to the robot's center.
            Yi (float): The y-coordinate of the wheel relative to the robot's center.

        Returns:
            float: The angle of the wheel in radians.
        """
        return atan2(Vy + omega * Xi, Vx - omega * Yi)
    
    def compute(
        self,
        robotVelocity: Tuple[float, float, float], # This is the x, y, and z velocity of the robot
        robot_angular_velocity: float,
        robotAngle,
    ) -> Tuple[float, float]:
        """
        Compute the angle and speed of a wheel in a swerve drive system.
        
        This method calculates the angle and speed of a wheel based on the given velocities and
        the position of the wheel relative to the robot's center.
        
        Args:
            robotVelocity (Tuple[float, float, float]): The velocity of the robot along the x, y, and z axes.
            robot_angular_velocity (float): The angular velocity of the robot.
            robotAngle (float): The angle of the robot in radians.
            
        Returns:
            Tuple[float, float]: The angle and speed of the wheel.
        """
        
        
        rotatedX = self.module_position.get("x") * cos(robotAngle) - self.module_position.get("y") * sin(robotAngle)
        rotatedY = self.module_position.get("x") * sin(robotAngle) + self.module_position.get("y") * cos(robotAngle)

        # Calculate the wheel angle and speed
        wheel_speed = self.compute_wheel_speed(robotVelocity[0], robotVelocity[1], robot_angular_velocity, rotatedX, rotatedY) / self.wheel_size
        wheel_angle = self.shortest_angle(self.angle,  self.compute_wheel_angle(robotVelocity[0], robotVelocity[1], robot_angular_velocity, rotatedX, rotatedY) - robotAngle)
        self.angle = wheel_angle
        return [round(wheel_angle, 2), round(wheel_speed, 2)]
    
