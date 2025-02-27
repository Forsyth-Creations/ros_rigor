# This component will subscribe to four
# swerve drive modules, and then pull their data into a
# joint state message. This message will be published to the
# joint state publisher

# include the other packages
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64, Int8
from nav_msgs.msg import Odometry
import tf2_ros
from geometry_msgs.msg import TransformStamped, Twist
from math import sin, cos
from typing import Tuple
from rclpy.duration import Duration
from math import atan2

# import the swerve module


# Create the Robot Class
class ConnectionManager:
    def __init__(self):
        self.pivot_publishers: list = []
        self.drive_publishers: list = []
        self.mode_publishers: list = []


class ModuleKinematics:
    def __init__(self, name, wheel_size, module_position : dict):
        self.name = name
        self.wheel_size = wheel_size
        self.module_position : dict = module_position

    def compute(
        self,
        robotPosition: Tuple[float, float, float],
        robotVelocity: Tuple[float, float, float],
        robotAngle,
    ):
        rotatedX = self.module_position.get("x") * cos(robotAngle) - self.module_position.get("y") * sin(robotAngle)
        rotatedY = self.module_position.get("x") * sin(robotAngle) + self.module_position.get("y") * cos(robotAngle)

        # Calculate the wheel angle and speed
        wheel_speed = self.compute_wheel_speed(robotVelocity[0], robotVelocity[1], robotVelocity[2], rotatedX, rotatedY)
        wheel_angle = self.compute_wheel_angle(robotVelocity[0], robotVelocity[1], robotVelocity[2], rotatedX, rotatedY)
        
        # 90 degree offset to radians
        # wheel_angle -= 1.5708
                
        return [wheel_angle, wheel_speed]
    
    def compute_wheel_speed(self, Vx, Vy, omega, Xi, Yi):
        return ( (Vx - omega * Yi) ** 2 + (Vy + omega * Xi) ** 2 ) ** 0.5

    def compute_wheel_angle(self, Vx, Vy, omega, Xi, Yi):
        # 90 degree offset to radians
        offset = 1.5708
        return atan2(Vy + omega * Xi, Vx - omega * Yi) - offset


class Robot(Node):
    def __init__(self, name):
        # Initialize the node
        super().__init__("hermes")

        self.odom_frequency = 10.0

        self.get_logger().info(f"Creating Module: {name}")

        self.module_name = (
            self.declare_parameter("module_name", "hermes")
            .get_parameter_value()
            .string_value
        )
        self.connections = ConnectionManager()

        # Create a parameter for an array of swerve module names
        self.declare_parameter(
            "module_topic_prefixes", "swerve_a,swerve_b,swerve_c,swerve_d"
        )

        # Get the parameter value (which will be a comma-separated string)
        module_topic_prefixes = self.get_parameter("module_topic_prefixes").value

        # Split the string into a list of prefixes
        self.module_topic_prefixes = module_topic_prefixes.split(",")

        # Make some pivot_publishers
        for module in self.module_topic_prefixes:
            self.pivot_position_pub = self.create_publisher(
                Float64, f"/{module}/rqst_pivot_direction", 10
            )
            self.connections.pivot_publishers.append(self.pivot_position_pub)

        # Make some drive publishers
        for module in self.module_topic_prefixes:
            self.drive_speed_pub = self.create_publisher(
                Float64, f"/{module}/rqst_wheel_speed", 10
            )
            self.connections.drive_publishers.append(self.drive_speed_pub)

        self.get_logger().info(
            f"Creating Robot with Modules: {self.module_topic_prefixes}"
        )

        # Wait for tf to start
        self.get_logger().info("Waiting for tf...")
        # Set up the duration object
        duration = Duration(seconds=1)  # Equivalent to 1 second
        self.tfBuffer = tf2_ros.Buffer(duration, self)
        self.listener = tf2_ros.TransformListener(self.tfBuffer, self)
        self.get_logger().info("tf is ready")

        # Create some things for odom
        self.odom_pub = self.create_publisher(Odometry, "odom", 10)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        self.timer = self.create_timer(
            1 / self.odom_frequency, self.update_odometry
        )  # Based on the odom frequency

        self.wheel_positions = [0.0, 0.0, 0.0, 0.0]
        self.pivot_positions = [0.0, 0.0, 0.0, 0.0]

        self.previous_avg_wheel_speed = 0.0
        self.get_clock().use_sim_time = True

        # Subscribe to /swerve_c/pivot_position and /swerve_c/wheel_speed topics
        for prefix in self.module_topic_prefixes:
            self.create_subscription(
                Float64,
                f"/{prefix}/pivot_position",
                self.pivot_position_callback(prefix),
                10,
            )
            self.create_subscription(
                Float64, f"/{prefix}/wheel_position", self.wheel_speed_callback(prefix), 10
            )

        # Subscribe to the command velocity topic
        self.positions = {"x": 0.0, "y": 0.0, "th": 0.0}
        self.commanded_vel = Twist()
        self.cmd_vel_sub = self.create_subscription(
            Twist, f"/{self.module_name}/cmd_vel", self.set_cmd_vel, 10
        )

        # TODO: Create all the module helpers, use the TF frame to get the position of the modules
        self.module_helpers: list[ModuleKinematics] = []

        spacing = 0.1524
        offsets = [
            {"x": spacing, "y": spacing},
            {"x": spacing, "y": -spacing},
            {"x": -spacing, "y": -spacing},
            {"x": -spacing, "y": spacing}
        ]

        for idx, prefix in enumerate(self.module_topic_prefixes):
            self.get_logger().info(f"Creating Module Helper for {prefix}")
            self.module_helpers.append(ModuleKinematics(prefix, 0.1, offsets[idx]))

    def set_cmd_vel(self, msg):
        # Set commanded to the message
        self.commanded_vel = msg
        self.commanded_vel.linear.x /= 10
        self.commanded_vel.linear.y /= 10

    def wheel_speed_callback(self, prefix):
        def callback(msg):
            self.wheel_positions[self.module_topic_prefixes.index(prefix)] = msg.data

        return callback

    def pivot_position_callback(self, prefix):
        def callback(msg):
            self.pivot_positions[self.module_topic_prefixes.index(prefix)] = msg.data

        return callback

    def update_odometry(self):

        # Note: this algorithm is based on the work done here:
        # https://www.forsythcreations.com/swerve_drive

        odom = Odometry()
        t = TransformStamped()
        dt = 1 / self.odom_frequency
        
        # Compute the wheel speed and angle for each module
        wheel_speeds = []
        wheel_angles = []
        for idx, helper in enumerate(self.module_helpers):
            wheel_angle, wheel_speed = helper.compute(
                (self.positions.get("x", 0.0), self.positions.get("y", 0.0), self.positions.get("th", 0.0)),
                (getattr(self.commanded_vel.linear, 'x', 0.0), getattr(self.commanded_vel.linear, 'y', 0.0), getattr(self.commanded_vel.angular, 'z', 0.0)),
                self.positions.get("th", 0.0)
            )
            wheel_speeds.append(wheel_speed)
            wheel_angles.append(wheel_angle)
            
        # Publish the wheel speeds and pivot positions
        for idx, prefix in enumerate(self.module_topic_prefixes):
            self.connections.drive_publishers[idx].publish(Float64(data=float(wheel_speeds[idx])))
            self.connections.pivot_publishers[idx].publish(Float64(data=float(wheel_angles[idx])))

        # Calculate the average wheel speed and pivot position
        xt1 = self.positions.get("x", 0.0) + getattr(self.commanded_vel.linear, 'x', 0.0) * dt
        yt1 = self.positions.get("y", 0.0) + getattr(self.commanded_vel.linear, 'y', 0.0) * dt
        tht1 = self.positions.get("th", 0.0) + getattr(self.commanded_vel.angular, 'z', 0.0) * dt

        # Update the positions
        self.positions["x"] = xt1
        self.positions["y"] = yt1
        self.positions["th"] = tht1

        # -----------------------------------------------------------------------------

        # Create the pose
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_footprint"
        odom.pose.pose.position.x = self.positions.get("x", 0.0)
        odom.pose.pose.position.y = self.positions.get("y", 0.0)
        odom.pose.pose.position.z = 0.0

        # Create a quaternion from the yaw angle
        odom.pose.pose.orientation.x = 0.0
        odom.pose.pose.orientation.y = 0.0
        odom.pose.pose.orientation.z = 0.0
        odom.pose.pose.orientation.w = 0.0

        # Publish the message
        self.odom_pub.publish(odom)

        # Create a transform message
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "odom"
        t.child_frame_id = "base_footprint"
        t.transform.translation.x = self.positions.get("x", 0.0)
        t.transform.translation.y = self.positions.get("y", 0.0)
        t.transform.translation.z = self.positions.get("z", 0.0)
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = sin(self.positions.get("th", 0.0) / 2)
        t.transform.rotation.w = cos(self.positions.get("th", 0.0) / 2)

        # Publish the transform
        self.tf_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)

    robot = Robot(name="hermes_robot_node")

    # Spin to keep the node active
    try:
        rclpy.spin(robot)
    except KeyboardInterrupt:
        pass

    # Shutdown when finished
    robot.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
