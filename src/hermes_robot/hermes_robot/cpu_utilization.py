import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

import psutil

def get_cpu_utilization():
    """
    Get the CPU utilization percentage.
    """
    try:
        # Get CPU utilization percentage
        cpu_utilization = psutil.cpu_percent(interval=1)
        return cpu_utilization
    except Exception as e:
        print(f"Failed to get CPU utilization: {e}")
        return None
    
class CPUUtilizationNode(Node):
    def __init__(self):
        super().__init__('cpu_utilization_node')
        self.publisher_ = self.create_publisher(Float32, 'cpu_utilization', 10)
        self.timer = self.create_timer(1.0, self.publish_cpu_utilization)  # Publish at 1 Hz

    def publish_cpu_utilization(self):
        cpu_utilization = get_cpu_utilization()
        if cpu_utilization is not None:
            msg = Float32()
            msg.data = cpu_utilization
            self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = CPUUtilizationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
