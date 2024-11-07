# This component will subscribe to four
# swerve drive modules, and then pull their data into a 
# joint state message. This message will be published to the
# joint state publisher

# include the other packages
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState

# import the swerve module
from hermes_swerve_module.hermes_swerve_module.swerve_module import Module

# Create the Robot Class

class Robot(Node):
    def __init__(self):
        # Initialize the node
        super().__init__('robot_node')
        
        # Create the Joint State Publisher
        self.joint_state_pub = self.create_publisher(JointState, '/joint_states', qos_profile_sensor_data)
        
        # Create the swerve modules
        self.swerve_modules = [Module('front_left'), Module('front_right'), Module('back_left'), Module('back_right')]
        
        # Create the timer
        self.timer = self.create_timer(0.1, self.publish_joint_state)
        
    def publish_joint_state(self):
        # Create the joint state message
        joint_state_msg = JointState()
        joint_state_msg.header.stamp = self.get_clock().now().to_msg()

        # Loop through the swerve modules
        joint_state_msg.name = []
        joint_state_msg.position = []
        joint_state_msg.velocity = []
        joint_state_msg.effort = []
        
        for module in self.swerve_modules:
            joint_state_msg.name.append(module.module_name)
            joint_state_msg.position.append(module.commanded_pivot_position)
            joint_state_msg.velocity.append(0)
            joint_state_msg.effort.append(0)
            
        # Publish the joint state message
        self.joint_state_pub.publish(joint_state_msg)
        self.get_logger().info('Published Joint State')
    
def main(args=None):
    import rclpy
    
    rclpy.init(args=args)
    robot = Robot()

    # Spin the node to keep it running
    rclpy.spin(robot)

    # Shutdown when finished
    robot.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()