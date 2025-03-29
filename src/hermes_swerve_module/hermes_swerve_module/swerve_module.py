import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState
from hermes_swerve_module.pid import PIDController
from geometry_msgs.msg import Twist

CSI = '\033['

def code_to_chars(code):
    return CSI + str(code) + 'm'
class AnsiCodes(object):
    def __init__(self):
        # the subclasses declare class attributes which are numbers.
        # Upon instantiation we define instance attributes, which are the same
        # as the class attributes but wrapped with the ANSI escape sequence
        for name in dir(self):
            if not name.startswith('_'):
                value = getattr(self, name)
                setattr(self, name, code_to_chars(value))
                
class AnsiFore(AnsiCodes):
    BLACK           = 30
    RED             = 31
    GREEN           = 32
    YELLOW          = 33
    BLUE            = 34
    MAGENTA         = 35
    CYAN            = 36
    WHITE           = 37
    RESET           = 39

Fore   = AnsiFore()

class Module(Node):
    def __init__(self, name):
        
        """
        For this, I've made a distinction between actual, target, and commanded values
        
        Actual: The actual value of the hardware
        Requesting: What the user wants to set the value to
        Commanding: What the controller is commanding the hardware to do
        
        """
        
        # Use the namespace parameter
        super().__init__(name)
        
        self.odom_frequency = 10.0
        
        self.warning_flag_for_dime = False
        self.module_name = self.declare_parameter('module_name', 'module_default').get_parameter_value().string_value
        self.mock_encoder_values = self.declare_parameter('mock_encoder_values', True).get_parameter_value().bool_value # For running without actual hardware
        self.invert_drive_motor = self.declare_parameter('invert_drive_motor', False).get_parameter_value().bool_value # For running without actual hardware
        # self.invert_drive_motor = True
        self.get_logger().info(f'Creating Module: {name}')
        
        # Print out that the invert_drive_motor parameter is set
        if self.invert_drive_motor:
            self.get_logger().info(f'{Fore.GREEN}Inverting Drive Motor for {self.module_name}{Fore.RESET}')
        
        # Publish Encoder values and limit switch state
        self.encoder_sub = self.create_publisher(JointState, f'/{self.module_name}/encoders', 10)
        self.limit_switch_sub = self.create_publisher(Float64, f'/{self.module_name}/limit_switch', 10)

        # Pub/Sub for target pivot angle
        # self.rqst_pivot_sub = self.create_subscription(Float64, f'/{self.module_name}/rqst_pivot_angle', self.set_rqst_pivot_angle, 10)
        self.pivot_position_pub = self.create_publisher(Float64, f'/{self.module_name}/pivot_position', 10)
        
        # Move all request commands to a common "Twist" command, where X is the speed and Z is the pivot angle
        self.command_sub = self.create_subscription(Twist, f'/{self.module_name}/command', self.command_callback, 10)
        
        # Pub/Sub for motor speed
        # self.drive_motor_sub = self.create_subscription(Float64, f'/{self.module_name}/rqst_wheel_speed', self.set_rqst_wheel_speed, 10)
        self.drive_motor_pub = self.create_publisher(Float64, f'/{self.module_name}/wheel_position', 10)
        self.speed_motor_pub = self.create_publisher(Float64, f'/{self.module_name}/wheel_speed', 10)
        
        # Create an error publisher for the pivot direction
        self.pivot_error_pub = self.create_publisher(Float64, f'/{self.module_name}/pivot_error', 10)
        
        # Publish the error at an interval (compute_pivot_error)
        self.error_timer = self.create_timer(
            1 / 2,  # Publish every 2 seconds to reduce CPU usage
            self.compute_pivot_error
        )
        self.pivot_error = 0.0  # Initialize pivot error

        # Internal state variables for module
        self.actual_wheel_speed = 0.0    # actual drive velocity from encoder
        self.rqst_wheel_speed = 0.0    # rqst drive velocity
        self.drive_wheel_position = 0.0
        self.target_wheel_position = 0.0
        

        self.rqst_pivot_angle = 0.0    # rqst pivot angle
        self.actual_pivot_angle = 0.0    # actual pivot angle from encoder
        
        
        self.commanded_wheel_speed = 0.0  # commanded drive velocity
        self.commanded_pivot_position = 0.0  # commanded pivot angle
        
        self.limit_switch_triggered = False
        
        # PID Controller for the drive
        self.drive_pid_controller = PIDController(p=0.1, i=0.4, d=0.01, target=self.rqst_wheel_speed, max_output=2, min_output=-2, logger=self.get_logger())

        # PID Controller for the pivot
        self.pivot_pid_controller = PIDController(p=0.1, i=0.1, d=0.01, target=self.rqst_pivot_angle, max_output=6.28, min_output=0.0, logger=self.get_logger())
        
        # Occassionally update the positions based on the differnentials
        self.timer = self.create_timer(
            1 / self.odom_frequency, self.update_position
        )
                
    def command_callback(self, msg):
        # Set the requested wheel speed and pivot angle based on the Twist message
        self.set_rqst_pivot_angle(msg.angular.z)    # Use the angular.z for pivot angle, add offset to match the simulation
        self.compute_pivot_error()                  # Compute the pivot error after setting the angle
        self.set_rqst_wheel_speed(msg.linear.x)     # Use the linear.x for wheel speed
        
    def compute_pivot_error(self):
        error = self.rqst_pivot_angle - self.actual_pivot_angle
        
        # Publish the pivot error for debugging purposes
        pivot_error_msg = Float64()
        pivot_error_msg.data = error
        self.pivot_error = error
        self.pivot_error_pub.publish(pivot_error_msg)
            

    def set_rqst_wheel_speed(self, speed : float):
        # Method to set drive speed
        if (self.pivot_error > 0.1):
            self.rqst_wheel_speed = self.rqst_wheel_speed * 0.95 # Reduce speed if the pivot error is too high to try and correct it
        else:
            self.rqst_wheel_speed = speed

    def set_rqst_pivot_angle(self, angle : float):
        # Method to set pivot angle
        self.rqst_pivot_angle = angle

    def update_encoder_values(self):
        # Callback to receive encoder feedback
        if not self.mock_encoder_values:
            # Get the actual encoder values
            raise NotImplementedError('Actual encoder values not implemented yet')
        else:
            # Mock encoder values for testing without actual hardware, this means that the encoder values match the commanded values
            self.actual_wheel_speed = self.commanded_wheel_speed
            self.actual_pivot_angle = self.commanded_pivot_position
            

    def update_limit_switch(self):
        # Callback to receive limit switch state
        if not self.mock_encoder_values:
            # Get the actual limit switch state
            raise NotImplementedError('Actual limit switch state not implemented yet')
        else:
            # Mock limit switch state for testing without actual hardware
            self.limit_switch_triggered = False

    def update_position(self):
        # log some values for me to see
        # self.get_logger().info(f'{self.module_name}: RPA: {self.rqst_pivot_angle} | AP: {self.actual_pivot_angle} | RWS: {self.rqst_wheel_speed} | AWS: {self.actual_wheel_speed}')
        
        self.update_encoder_values()
        self.update_limit_switch()
        
        # -------------------------- Driving Wheel --------------------------------------
        drive_output = self.drive_pid_controller.compute_move(self.rqst_wheel_speed, self.actual_wheel_speed)
        
        # Publish the wheel speed
        speed_msg = Float64()
        speed_msg.data = self.actual_wheel_speed
        self.speed_motor_pub.publish(speed_msg)
        
        self.drive_wheel_position += drive_output if not self.invert_drive_motor else -drive_output
        # self.get_logger().info(f'{self.module_name}: Drive Wheel Position: {self.drive_wheel_position} | Requested Wheel Speed: {self.rqst_wheel_speed} | Actual Wheel Speed: {self.actual_wheel_speed}')
        
        drive_msg = Float64()
        drive_msg.data = self.drive_wheel_position
        
        # consider self.self.invert_drive_motor
        self.drive_motor_pub.publish(drive_msg)
        
        # -------------------------- Pivot Wheel --------------------------------------
        modified_rqst_pivot_angle = self.rqst_pivot_angle
        pivot_output = self.pivot_pid_controller.compute_move(modified_rqst_pivot_angle, self.actual_pivot_angle)
        pivot_msg = Float64()
        pivot_msg.data = pivot_output # add the offset to the pivot angle to keep it synced with the sim 
        self.pivot_position_pub.publish(pivot_msg)
        
        # if we are mocking the hardware, then the actual values are the commanded values
        if self.mock_encoder_values:
            self.commanded_wheel_speed = drive_output
            self.commanded_pivot_position = pivot_output
            
def main(args=None):
    rclpy.init(args=args)

    # Instantiate the module with the specified namespace
    module = Module(name='hermes_swerve_module')

    # Spin to keep the node active
    try:
        rclpy.spin(module)
    except KeyboardInterrupt:
        pass

    # Shutdown the node
    module.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
