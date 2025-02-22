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
from geometry_msgs.msg import TransformStamped
from math import sin, cos
from rclpy.duration import Duration

# import the swerve module

# Create the Robot Class
class ConnectionManager():
    def __init__(self):
        self.pivot_publishers : list = []
        self.drive_publishers : list = []
        self.mode_publishers : list = []

class Robot(Node):
    def __init__(self, name):
        # Initialize the node
        super().__init__('hermes')
        
        self.get_logger().info(f'Creating Module: {name}')
        
        self.module_name = self.declare_parameter('module_name', 'hermes').get_parameter_value().string_value
        self.connections = ConnectionManager()
        
        # Create a parameter for an array of swerve module names
        self.declare_parameter('module_topic_prefixes', 'swerve_a,swerve_b,swerve_c,swerve_d')

        # Get the parameter value (which will be a comma-separated string)
        module_topic_prefixes = self.get_parameter('module_topic_prefixes').value

        # Split the string into a list of prefixes
        self.module_topic_prefixes = module_topic_prefixes.split(',')
        
        # Subscribe to the pivot position topic
        self.rqst_pivot_position = 0.0
        self.rqst_pivot_sub = self.create_subscription(Float64, f'/{self.module_name}/rqst_pivot_direction', self.set_rqst_pivot_direction, 10)
        
        # Make some pivot_publishers
        for module in self.module_topic_prefixes:
            self.pivot_position_pub = self.create_publisher(Float64, f'/{module}/rqst_pivot_direction', 10)
            self.connections.pivot_publishers.append(self.pivot_position_pub)
            
        # Subscribe to the wheel speed topic
        self.rqst_wheel_speed = 0.0
        self.rqst_wheel_sub = self.create_subscription(Float64, f'/{self.module_name}/rqst_wheel_speed', self.set_rqst_wheel_speed, 10)
            
        # Make some drive publishers
        for module in self.module_topic_prefixes:
            self.drive_speed_pub = self.create_publisher(Float64, f'/{module}/rqst_wheel_speed', 10)
            self.connections.drive_publishers.append(self.drive_speed_pub)
            
        self.get_logger().info(f'Creating Robot with Modules: {self.module_topic_prefixes}')
        
        # Wait for tf to start
        self.get_logger().info('Waiting for tf...')
        # Set up the duration object
        duration = Duration(seconds=1)  # Equivalent to 1 second
        self.tfBuffer = tf2_ros.Buffer(duration, self)
        self.listener = tf2_ros.TransformListener(self.tfBuffer, self)
        self.get_logger().info('tf is ready')
        
        # Create some things for odom
        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        self.timer = self.create_timer(0.1, self.update_odometry) # 10 Hz
        
        self.x = 0.0
        self.y = 0.0
        self.th = 0.0
        self.vx = 0.1  # Example linear velocity
        self.vth = 0.0  # Example angular velocity
        
        
        self.wheel_positions = [0.0, 0.0, 0.0, 0.0]
        self.pivot_positions = [0.0, 0.0, 0.0, 0.0]
        
        self.previous_avg_wheel_speed = 0.0
        self.get_clock().use_sim_time = True
        
        # Subscribe to /swerve_c/pivot_position and /swerve_c/wheel_speed topics
        for prefix in self.module_topic_prefixes:
            self.create_subscription(Float64, f'/{prefix}/pivot_position', self.pivot_position_callback(prefix), 10)
            self.create_subscription(Float64, f'/{prefix}/wheel_speed', self.wheel_speed_callback(prefix), 10)
        
        self.control_mode = 0
        self.mode_sub = self.create_subscription(Int8, f'/{self.module_name}/mode', self.mode_callback, 10)
        
        # Create the mode publishers
        for module in self.module_topic_prefixes:
            self.mode_pub = self.create_publisher(Int8, f'/{module}/rqst_mode', 10)
            self.connections.mode_publishers.append(self.mode_pub)
            
                    
    def mode_callback(self, msg):
        # Request the mode from the swerve modules
        self.control_mode = msg.data
        for module in self.connections.mode_publishers:
            module.publish(msg)
        
    def wheel_speed_callback(self, prefix):
        def callback(msg):
            self.wheel_positions[self.module_topic_prefixes.index(prefix)] = msg.data
        return callback
    
    def pivot_position_callback(self, prefix):
        def callback(msg):
            self.pivot_positions[self.module_topic_prefixes.index(prefix)] = msg.data
        return callback
        
    
    def update_odometry(self):
        dt = 0.1  # Timer period
        
        # Calculate the average wheel speed and pivot position
        avg_wheel_position = sum(self.wheel_positions) / len(self.wheel_positions)
        avg_pivot_position = sum(self.pivot_positions) / len(self.pivot_positions)
        
        # Calculate the average velocity
        avg_wheel_speed = (avg_wheel_position - self.previous_avg_wheel_speed) / dt
        self.previous_avg_wheel_speed = avg_wheel_position
        
        wheel_size = 0.18  # Diameter of the wheel in meters
        speed_considering_wheel_size = avg_wheel_speed * wheel_size
        odom = Odometry()
        t = TransformStamped()
        
        if (self.control_mode == 1): # on-a-dime movement
            # This will only impart a change in angular velocity
            
            
            # Calculate the angular velocity
            self.vth = -speed_considering_wheel_size * cos(avg_pivot_position)
            
            # Update pose
            self.th += self.vth * dt
            
            # Create the pose
            odom.header.stamp = self.get_clock().now().to_msg()
            odom.header.frame_id = 'odom'
            odom.child_frame_id = 'base_footprint'
            odom.pose.pose.position.x = self.x
            odom.pose.pose.position.y = self.y
            odom.pose.pose.position.z = 0.0
            
            # Create a quaternion from the yaw angle
            odom.pose.pose.orientation.x = 0.0
            odom.pose.pose.orientation.y = 0.0
            odom.pose.pose.orientation.z = sin(self.th / 2)
            odom.pose.pose.orientation.w = cos(self.th / 2)
            
            # Publish the message
            self.odom_pub.publish(odom)
            
            # Create a transform message
            t.header.stamp = self.get_clock().now().to_msg()
            t.header.frame_id = 'odom'
            t.child_frame_id = 'base_footprint'
            t.transform.translation.x = self.x
            t.transform.translation.y = self.y
            t.transform.translation.z = 0.0
            t.transform.rotation.x = 0.0
            t.transform.rotation.y = 0.0
            t.transform.rotation.z = sin(self.th / 2)
            t.transform.rotation.w = cos(self.th / 2)
            
            # Publish the transform
            self.tf_broadcaster.sendTransform(t)
            
        elif (self.control_mode == 0): # standard movement
            
            # Update pose
            self.x -= (speed_considering_wheel_size * sin(avg_pivot_position)) * dt
            self.y -= (speed_considering_wheel_size * cos(avg_pivot_position)) * dt
            # self.th += self.vth * dt
            self.th = 0
            
            # Create the pose
            odom.header.stamp = self.get_clock().now().to_msg()
            odom.header.frame_id = 'odom'
            odom.child_frame_id = 'base_footprint'
            odom.pose.pose.position.x = self.x
            odom.pose.pose.position.y = self.y
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
            t.header.frame_id = 'odom'
            t.child_frame_id = 'base_footprint'
            t.transform.translation.x = self.x
            t.transform.translation.y = self.y
            t.transform.translation.z = 0.0
            t.transform.rotation.x = 0.0
            t.transform.rotation.y = 0.0
            t.transform.rotation.z = sin(self.th / 2)
            t.transform.rotation.w = cos(self.th / 2)
            
            # Publish the transform
            self.tf_broadcaster.sendTransform(t)

    def set_rqst_pivot_direction(self, msg):
        self.rqst_pivot_position = msg.data
        for module in self.connections.pivot_publishers:
            module.publish(msg)
            
    def set_rqst_wheel_speed(self, msg):
        self.rqst_wheel_speed = msg.data
        for module in self.connections.drive_publishers:
            module.publish(msg)
            
def main(args=None):
    rclpy.init(args=args)
    
    robot = Robot(name='hermes_robot_node')
    
    # Spin to keep the node active
    try:
        rclpy.spin(robot)
    except KeyboardInterrupt:
        pass

    # Shutdown when finished
    robot.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()