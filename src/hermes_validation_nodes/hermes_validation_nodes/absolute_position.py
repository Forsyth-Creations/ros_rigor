import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray, PointStamped

class AbsolutePosition(Node):
    def __init__(self, name="absolute_gazebo_position"):
        super().__init__(name)
        
        # Declare parameters for topic names
        self.declare_parameter("ros_pos_array_topic", "/hermes/gazebo/pose")
        self.declare_parameter("position_topic_out", "/gazebo/absolute/position")
        self.declare_parameter("position_value_index", 0)  # Default index for selecting a pose from the array

        # Retrieve topic names
        ros_pos_array_topic = self.get_parameter("ros_pos_array_topic").value
        position_topic = self.get_parameter("position_topic_out").value
        position_value_index = self.get_parameter("position_value_index").value
        
        # Subscribe to the topic
        self.subscription = self.create_subscription(
            PoseArray,
            ros_pos_array_topic,
            self.callback_make_point,
            10
        )
        
        # Create a publisher for the Point message
        self.publisher = self.create_publisher(
            PointStamped,
            position_topic,
            10
        )
        
        self.position_index = position_value_index
        
        
    def callback_make_point(self, msg):
        # Convert a pose array to a single x and y point
        if not msg.poses or len(msg.poses) <= self.position_index:
            self.get_logger().warn("Received empty PoseArray.")
            return
        
        # Extract the first pose from the array
        pose = msg.poses[self.position_index]
        
        # Create a Point message
        point_msg = PointStamped()
        # Extract x, y, z coordinates from the pose
        point_msg.point.x = pose.position.y
        point_msg.point.y = -pose.position.x
        
        # Add in the timestamp
        point_msg.header.stamp = self.get_clock().now().to_msg()
        
        # Publish the Point message
        self.publisher.publish(point_msg)
    

def main(args=None):
    rclpy.init(args=args)
    node = AbsolutePosition()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Node interrupted by user. Shutting down...")
    except Exception as e:
        node.get_logger().error(f"An error occurred: {str(e)}")
                
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
