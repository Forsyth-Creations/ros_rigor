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
from math import sin, cos, atan2, pi, asin
from typing import Tuple
from rclpy.duration import Duration
from hermes_robot.swerve_math import ModuleKinematics

# IMU message type
from sensor_msgs.msg import Imu

# Create the Robot Class
class ConnectionManager:
    def __init__(self):
        self.pivot_publishers: list = []
        self.drive_publishers: list = []
        self.mode_publishers: list = []


class Robot(Node):
    def __init__(self, name):
        # Initialize the node
        super().__init__("hermes")

        self.odom_frequency = 20.0
        
        self.fatal_flag = False

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
                Float64, f"/{module}/rqst_pivot_angle", 10
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
            self.module_helpers.append(ModuleKinematics(prefix, 0.4, offsets[idx]))
            
        # Create a publisher for the robot angle
        self.robot_angle_pub = self.create_publisher(Float64, f"/{self.module_name}/robot_angle", 10)
        
        self.imu_data = Imu()
        
        # Subscribe to the hermes/imu topic
        self.create_subscription(Imu, "/hermes/imu", self.imu_callback, 10)
        
    def imu_callback(self, msg):
        # Get the orientation from the message
        orientation = msg.orientation
        roll, pitch, yaw = self.quaternion_to_euler(orientation.x, orientation.y, orientation.z, orientation.w)
        self.positions["th"] = yaw
        self.imu_data = msg
        
        # Determine if I have fallen over
        if abs(roll) > 0.5 or abs(pitch) > 0.5:
            self.get_logger().error("I have fallen over")
            self.commanded_vel = Twist()
            self.commanded_vel.linear.x = 0.0
            self.commanded_vel.linear.y = 0.0
            self.commanded_vel.angular.z = 0.0  # Stop the robot
            self.update_odometry()
            self.fatal_flag = True
        
        
    def quaternion_to_euler(self, x, y, z, w):
        # Convert a quaternion to euler angles
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        X = atan2(t0, t1)
        
        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        Y = asin(t2)
        
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        Z = atan2(t3, t4)
        
        return X, Y, Z

    def set_cmd_vel(self, msg):
        # Set commanded to the message
        if self.fatal_flag:
            return
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
        # Update the odometry based on the current mode
        self.wheel_based_update_odometry()
    
    def imu_based_update_odometry(self):
        robot_angle = Float64()
        robot_angle.data = self.imu_data.orientation.z
        self.robot_angle_pub.publish(robot_angle)
        
        # Use the imu data to update the odometry
        odom = Odometry()
        t = TransformStamped()
        
        # Pull the x and y velocities from the imu
        x_vel = self.imu_data.linear_acceleration.x
        y_vel = self.imu_data.linear_acceleration.y
        z_vel = self.imu_data.angular_velocity.z
        
        # Calculate the new x, y, and theta positions
        dt = 1 / self.odom_frequency
        
        xt1 = self.positions.get("x", 0.0) + x_vel * dt
        yt1 = self.positions.get("y", 0.0) + y_vel * dt
        tht1 = self.positions.get("th", 0.0) + z_vel * dt
        
        # Update the positions
        self.positions["x"] = xt1
        self.positions["y"] = yt1
        self.positions["th"] = tht1
        
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
        odom.pose.pose.orientation.z = sin(self.positions.get("th", 0.0) / 2)
        odom.pose.pose.orientation.w = cos(self.positions.get("th", 0.0) / 2)
        
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
        

    def wheel_based_update_odometry(self):

        # Note: this algorithm is based on the work done here:
        # https://www.forsythcreations.com/swerve_drive
        
        # Publish the current robot angle
        robot_angle = Float64()
        robot_angle.data = self.positions.get("th", 0.0)
        self.robot_angle_pub.publish(robot_angle)

        odom = Odometry()
        t = TransformStamped()
        dt = 1 / self.odom_frequency
        
        # Compute the wheel speed and angle for each module
        wheel_speeds = []
        wheel_angles = []
        for idx, helper in enumerate(self.module_helpers):
            wheel_angle, wheel_speed = helper.compute(
                (getattr(self.commanded_vel.linear, 'x', 0.0), getattr(self.commanded_vel.linear, 'y', 0.0), getattr(self.commanded_vel.angular, 'z', 0.0)),
                getattr(self.commanded_vel.angular, 'z', 0.0),
                self.positions.get("th", 0.0)
            )
            wheel_speeds.append(wheel_speed)
            wheel_angles.append(wheel_angle)
            
        # Publish the wheel speeds and pivot positions. TODO: The angle output is wrong and increasing (whcih is odd)
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
