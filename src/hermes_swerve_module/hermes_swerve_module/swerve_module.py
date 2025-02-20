import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64, Int8
from sensor_msgs.msg import JointState

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


# PID Controller Class
class PIDController:
    def __init__(self, p, i, d, target, max_output=2.0, min_output=-2.0, logger = None):
        self.p = p
        self.i = i
        self.d = d
        self.target = target
        self.max_output = max_output
        self.min_output = min_output
        self.prev_error = 0.0
        self.integral = 0.0
        self.logger = logger
        
    def get_logger(self):
        return self.logger

    def compute_move(self, requested_value, actual_value) -> float:
        """
        This function computes the PID output based on the actual value
        This function should be called at regular intervals to update the PID controller
        """
        error = requested_value - actual_value
        
        self.integral += error
        derivative = error - self.prev_error
        output = self.p * error + self.i * self.integral + self.d * derivative
        self.prev_error = error
        
        # round to 3 decimal places
        return round(output, 3)


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
        
        
        self.module_name = self.declare_parameter('module_name', 'module_default').get_parameter_value().string_value
        self.invert_drive_for_dime = self.declare_parameter('invert_drive_for_dime', False).get_parameter_value().bool_value
        self.dime_angle = self.declare_parameter('dime_angle', 0.785).get_parameter_value().double_value # The angle to go to when on a dime
        self.mock_encoder_values = self.declare_parameter('mock_encoder_values', True).get_parameter_value().bool_value # For running without actual hardware
        self.invert_drive_motor = self.declare_parameter('invert_drive_motor', False).get_parameter_value().bool_value # For running without actual hardware
        # self.invert_drive_motor = True
        self.get_logger().info(f'Creating Module: {name} with dime angle : {self.dime_angle}')
        
        # Print out that the invert_drive_motor parameter is set
        if self.invert_drive_motor:
            self.get_logger().info(f'{Fore.GREEN}Inverting Drive Motor for {self.module_name}{Fore.RESET}')
        
        # Publish Encoder values and limit switch state
        self.encoder_sub = self.create_publisher(JointState, f'/{self.module_name}/encoders', 10)
        self.limit_switch_sub = self.create_publisher(Float64, f'/{self.module_name}/limit_switch', 10)

        # Pub/Sub for target pivot angle
        self.rqst_pivot_sub = self.create_subscription(Float64, f'/{self.module_name}/rqst_pivot_direction', self.set_rqst_pivot_direction, 10)
        self.pivot_position_pub = self.create_publisher(Float64, f'/{self.module_name}/pivot_position', 10)
        
        # Pub/Sub for motor speed
        self.drive_motor_sub = self.create_subscription(Float64, f'/{self.module_name}/rqst_wheel_speed', self.set_rqst_wheel_speed, 10)
        self.drive_motor_pub = self.create_publisher(Float64, f'/{self.module_name}/wheel_speed', 10)

        # Internal state variables for module
        self.actual_wheel_speed = 0.0    # actual drive velocity from encoder
        self.actual_pivot_position = 0.0    # actual pivot angle from encoder
        
        self.rqst_wheel_speed = 0.0    # rqst drive velocity
        self.rqst_pivot_angle = 0.0    # rqst pivot angle
        self.drive_wheel_position = 0.0
        
        self.commanded_wheel_speed = 0.0  # commanded drive velocity
        self.commanded_pivot_position = 0.0  # commanded pivot angle
        
        self.limit_switch_triggered = False
        
        # PID Controller for the drive
        self.drive_pid_controller = PIDController(p=0.1, i=0.4, d=0.01, target=self.rqst_wheel_speed, max_output=2, min_output=-2, logger=self.get_logger())

        # PID Controller for the pivot
        self.pivot_pid_controller = PIDController(p=0.1, i=0.1, d=0.01, target=self.rqst_pivot_angle, max_output=6.28, min_output=0.0, logger=self.get_logger())
        
        
        # The mode handler for the module
        self.mode = 0
        self.mode_sub = self.create_subscription(Int8, f'/{self.module_name}/rqst_mode', self.set_mode, 10)
        self.mode_pub = self.create_publisher(Int8, f'/{self.module_name}/mode', 10)

    def set_mode(self, msg):
        # Method to set the mode of the module
        self.mode = msg.data
        self.mode_pub.publish(msg)
        
        if self.mode == 1:
            self.rqst_pivot_angle = self.dime_angle

    def set_rqst_wheel_speed(self, speed):
        # Method to set drive speed
        msg = Float64()
        msg.data = speed
        self.rqst_wheel_speed = speed.data / 5

    def set_rqst_pivot_direction(self, angle):
        # Method to set pivot angle
        msg = Float64()
        msg.data = angle
        # if in mode 1, only use the dime angle
        if self.mode == 1:
            # log the dime angle
            self.get_logger().warn(f'{Fore.YELLOW}{self.module_name}: Direct angle request ignored in Dime Mode{Fore.RESET}')
            self.rqst_pivot_angle = self.dime_angle
        else:
            self.rqst_pivot_angle = angle.data

    def update_encoder_values(self):
        # Callback to receive encoder feedback
        if not self.mock_encoder_values:
            # Get the actual encoder values
            raise NotImplementedError('Actual encoder values not implemented yet')
        else:
            # Mock encoder values for testing without actual hardware, this means that the encoder values match the commanded values
            self.actual_wheel_speed = self.commanded_wheel_speed
            self.actual_pivot_position = self.commanded_pivot_position
            

    def update_limit_switch_(self):
        # Callback to receive limit switch state
        if not self.mock_encoder_values:
            # Get the actual limit switch state
            raise NotImplementedError('Actual limit switch state not implemented yet')
        else:
            # Mock limit switch state for testing without actual hardware
            self.limit_switch_triggered = False


    def update(self):
        # Update method to be called regularly for control logic
        
        # log some values for me to see
        # self.get_logger().info(f'{self.module_name}: RPA: {self.rqst_pivot_angle} | AP: {self.actual_pivot_position} | RWS: {self.rqst_wheel_speed} | AWS: {self.actual_wheel_speed}')
        
        self.update_encoder_values()
        self.update_limit_switch_()
        
        # -------------------------- Driving Wheel --------------------------------------
        drive_output = self.drive_pid_controller.compute_move(self.rqst_wheel_speed, self.actual_wheel_speed)
        
        if (self.invert_drive_for_dime and self.mode == 1):
            self.drive_wheel_position -= drive_output if not self.invert_drive_motor else -drive_output
        else:
            self.drive_wheel_position += drive_output if not self.invert_drive_motor else -drive_output
        # self.get_logger().info(f'{self.module_name}: Drive Wheel Position: {self.drive_wheel_position} | Requested Wheel Speed: {self.rqst_wheel_speed} | Actual Wheel Speed: {self.actual_wheel_speed}')
        
        drive_msg = Float64()
        drive_msg.data = self.drive_wheel_position
        
        # consider self.self.invert_drive_motor
        self.drive_motor_pub.publish(drive_msg)
        
        # -------------------------- Pivot Wheel --------------------------------------
        modified_rqst_pivot_angle = max(-6.28, min(6.28, self.rqst_pivot_angle))
        pivot_output = self.pivot_pid_controller.compute_move(modified_rqst_pivot_angle, self.actual_pivot_position)
        pivot_msg = Float64()
        pivot_msg.data = pivot_output
        self.pivot_position_pub.publish(pivot_msg)
        
        # if we are mocking the hardware, then the actual values are the commanded values
        if self.mock_encoder_values:
            self.commanded_wheel_speed = drive_output
            self.commanded_pivot_position = pivot_output
            
def main(args=None):
    rclpy.init(args=args)

    # Instantiate the module with the specified namespace
    module = Module(name='hermes_swerve_module')

    # Use a timer to update the module
    timer_period = .1  # seconds
    module.create_timer(timer_period, module.update)

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
