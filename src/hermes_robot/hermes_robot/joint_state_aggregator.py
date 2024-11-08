import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32  # Assuming the incoming messages are of type Float32

class JointStateAggregator(Node):
    def __init__(self):
        super().__init__('joint_state_aggregator')

        # Create the joint state publisher
        self.joint_state_pub = self.create_publisher(JointState, 'joint_states', 10)
        
        # Initialize the dictionary to store the latest float values for each joint, with 0 as default
        self.received_joint_states = {
            'Joint_Revolve1': 0.0,
            'Joint_Drive1': 0.0,
            'Joint_Revolve2': 0.0,
            'Joint_Drive2': 0.0,
            'Joint_Revolve3': 0.0,
            'Joint_Drive3': 0.0,
            'Joint_Revolve4': 0.0,
            'Joint_Drive4': 0.0
        }

        # Map topics to joint names
        self.topic_to_joint_map = {
            '/swerve_a/pivot_position': 'Joint_Revolve1',
            '/swerve_a/wheel_speed': 'Joint_Drive1',
            '/swerve_b/pivot_position': 'Joint_Revolve2',
            '/swerve_b/wheel_speed': 'Joint_Drive2',
            '/swerve_c/pivot_position': 'Joint_Revolve3',
            '/swerve_c/wheel_speed': 'Joint_Drive3',
            '/swerve_d/pivot_position': 'Joint_Revolve4',
            '/swerve_d/wheel_speed': 'Joint_Drive4'
        }

        # Create subscriptions for each topic using a lambda to pass the topic name to the callback
        for topic, joint_name in self.topic_to_joint_map.items():
            self.create_subscription(
                Float32,
                topic,
                lambda msg, joint_name=joint_name: self.callback(msg, joint_name),
                10
            )
        
        # Create a timer to periodically publish aggregated joint states
        self.timer_period = 0.1  # seconds
        self.timer = self.create_timer(self.timer_period, self.publish_if_ready)

        # Publish initial 0 values
        self.publish_aggregated_joint_state()

    def callback(self, msg, joint_name):
        # Update the received joint state value for the specific joint
        self.received_joint_states[joint_name] = msg.data

    def publish_if_ready(self):
        # Check if all values have been received (values are initialized to 0 on startup)
        if all(state is not None for state in self.received_joint_states.values()):
            self.publish_aggregated_joint_state()

    def publish_aggregated_joint_state(self):
        # Combine all received float values into a JointState message
        aggregated_msg = JointState()
        aggregated_msg.header.stamp = self.get_clock().now().to_msg()

        aggregated_msg.name = list(self.received_joint_states.keys())
        aggregated_msg.position = [self.received_joint_states[joint] for joint in aggregated_msg.name]

        # Publish the aggregated joint state
        self.joint_state_pub.publish(aggregated_msg)

        # Clear the state for the next round
        for key in self.received_joint_states:
            self.received_joint_states[key] = None

def main(args=None):
    rclpy.init(args=args)
    
    joint_state_aggregator = JointStateAggregator()
    
    try:
        rclpy.spin(joint_state_aggregator)
    except KeyboardInterrupt:
        pass
    finally:
        joint_state_aggregator.destroy_node()
        rclpy.shutdown()
