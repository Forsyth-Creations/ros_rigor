import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64
import math

class TwistTransformer(Node):
    def __init__(self):
        super().__init__('twist_transformer')
        self.subscription_twist = self.create_subscription(
            Twist, '/cmd_vel_nav', self.twist_callback, 10)
        self.subscription_angle = self.create_subscription(
            Float64, '/hermes/robot_angle', self.angle_callback, 10)
        self.publisher = self.create_publisher(Twist, '/new_cmd_vel_nav', 10)
        self.latest_angle = 0.0

    def twist_callback(self, msg):
        hypotenuse = (msg.linear.x)  # Interpret x as hypotenuse
        
        ninety_degrees = math.pi / 2
        
        proper_angle = self.latest_angle + ninety_degrees
        
        x_component = hypotenuse * math.cos(proper_angle)
        y_component = hypotenuse * math.sin(proper_angle)
        
        new_twist = Twist()
        new_twist.linear.x = x_component
        new_twist.linear.y = y_component
        new_twist.linear.z = msg.linear.z
        new_twist.angular = msg.angular  # Preserve angular velocity
        
        # Get the logger and print the new twist
        # self.get_logger().info(f"New twist: x : {new_twist.linear.x}, y : {new_twist.linear.y}, z : {new_twist.linear.z} from angle {proper_angle}")
        
        self.publisher.publish(new_twist)

    def angle_callback(self, msg):
        self.latest_angle = msg.data  # Update angle from topic


def main():
    rclpy.init()
    node = TwistTransformer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()