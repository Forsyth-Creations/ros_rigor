import rclpy
from rclpy.node import Node
import importlib

class PersistentPublisher(Node):
    def __init__(self):
        super().__init__('persistent_publisher')
        self.declare_parameter('input_topic', 'input_topic')
        self.declare_parameter('output_topic', 'output_topic')
        self.declare_parameter('message_type', 'std_msgs.msg.Float32')
        self.declare_parameter('publish_frequency', 10.0)
        
        self.input_topic = self.get_parameter('input_topic').get_parameter_value().string_value
        self.output_topic = self.get_parameter('output_topic').get_parameter_value().string_value
        self.message_type_str = self.get_parameter('message_type').get_parameter_value().string_value
        self.publish_frequency = self.get_parameter('publish_frequency').get_parameter_value().double_value
        
        message_module, message_class = self.message_type_str.rsplit('.', 1)
        self.message_type = getattr(importlib.import_module(message_module), message_class)
        
        self.subscription = self.create_subscription(
            self.message_type, self.input_topic, self.listener_callback, 10)
        self.publisher = self.create_publisher(self.message_type, self.output_topic, 10)
        
        self.latest_value = None
        self.timer = self.create_timer(1.0 / self.publish_frequency, self.publish_latest_value)
        
    def listener_callback(self, msg):
        self.latest_value = msg
        
        # if msg.header.stamp == self.get_clock().now().to_msg():
        # See if the msg.header.stamp exists
        
        self.get_logger().info(f'Received new value: {msg}')

    def publish_latest_value(self):
        if self.latest_value is not None:
            if hasattr(self.latest_value, 'header') and hasattr(self.latest_value.header, 'stamp'): # This will redo the header timestamp to the current time
                self.latest_value.header.stamp = self.get_clock().now().to_msg()
            
            self.publisher.publish(self.latest_value)


def main(args=None):
    rclpy.init(args=args)
    node = PersistentPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()