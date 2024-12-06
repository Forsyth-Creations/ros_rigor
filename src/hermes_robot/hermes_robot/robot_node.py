# This component will subscribe to four
# swerve drive modules, and then pull their data into a 
# joint state message. This message will be published to the
# joint state publisher

# include the other packages
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64

# import the swerve module

# Create the Robot Class
class ConnectionManager():
    def __init__(self):
        self.pivot_publishers : list = []
        self.drive_publishers : list = []

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