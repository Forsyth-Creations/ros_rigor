import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

file_name = "/sys/class/thermal/thermal_zone0/temp"

def read_temperature():
    try:
        with open(file_name, 'r') as file:
            temp_str = file.read().strip()
            return float(temp_str) / 1000.0  # Convert millidegree Celsius to degree Celsius
    except Exception as e:
        print(f"Failed to read temperature: {e}")
        return None

class TemperatureNode(Node):
    def __init__(self):
        super().__init__('temperature_node')
        self.publisher_ = self.create_publisher(Float32, 'temperature', 10)
        self.timer = self.create_timer(1.0, self.publish_temperature)  # Publish at 1 Hz

    def publish_temperature(self):
        temperature = read_temperature()
        if temperature is not None:
            msg = Float32()
            msg.data = temperature
            self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = TemperatureNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()