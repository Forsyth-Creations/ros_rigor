import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from sensor_msgs.msg import JointState

# PID Controller Class
class PIDController:
    def __init__(self, p, i, d, target, max_output=2.0, min_output=-0.0):
        self.p = p
        self.i = i
        self.d = d
        self.target = target
        self.max_output = max_output
        self.min_output = min_output
        self.prev_error = 0.0
        self.integral = 0.0

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
        return round(max(self.min_output, min(self.max_output, output)), 3)


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
        
        self.get_logger().info(f'Creating Module: {name}')
        
        self.module_name = self.declare_parameter('module_name', 'module_default').get_parameter_value().string_value
        self.mock_encoder_values = self.declare_parameter('mock_encoder_values', True).get_parameter_value().bool_value # For running without actual hardware
        self.invert_drive_motor = self.declare_parameter('invert_drive_motor', False).get_parameter_value().bool_value # For running without actual hardware
        # self.invert_drive_motor = True
        
        # Publish Encoder values and limit switch state
        self.encoder_sub = self.create_publisher(JointState, f'/{self.module_name}/encoders', 10)
        self.limit_switch_sub = self.create_publisher(Float32, f'/{self.module_name}/limit_switch', 10)

        # Pub/Sub for target pivot angle
        self.rqst_pivot_sub = self.create_subscription(Float32, f'/{self.module_name}/rqst_pivot_direction', self.set_rqst_pivot_direction, 10)
        self.pivot_position_pub = self.create_publisher(Float32, f'/{self.module_name}/pivot_position', 10)
        
        # Pub/Sub for motor speed
        self.drive_motor_sub = self.create_subscription(Float32, f'/{self.module_name}/rqst_wheel_speed', self.set_rqst_wheel_speed, 10)
        self.drive_motor_pub = self.create_publisher(Float32, f'/{self.module_name}/wheel_speed', 10)

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
        self.drive_pid_controller = PIDController(p=0.1, i=0.4, d=0.01, target=self.rqst_wheel_speed, max_output=2, min_output=-2)

        # PID Controller for the pivot
        self.pivot_pid_controller = PIDController(p=0.1, i=0.1, d=0.01, target=self.rqst_pivot_angle, max_output=6.28, min_output=0.0)
        

    def set_rqst_wheel_speed(self, speed):
        # Method to set drive speed
        msg = Float32()
        msg.data = speed
        self.rqst_wheel_speed = speed.data

    def set_rqst_pivot_direction(self, angle):
        # Method to set pivot angle
        msg = Float32()
        msg.data = angle
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
        self.update_encoder_values()
        self.update_limit_switch_()
        
        # -------------------------- Driving Wheel --------------------------------------
        drive_output = self.drive_pid_controller.compute_move(self.rqst_wheel_speed, self.actual_wheel_speed)
        
        self.drive_wheel_position += drive_output if not self.invert_drive_motor else -drive_output
        
        # self.get_logger().info(f'{self.module_name}: Drive Wheel Position: {self.drive_wheel_position} | Requested Wheel Speed: {self.rqst_wheel_speed} | Actual Wheel Speed: {self.actual_wheel_speed}')
        
        drive_msg = Float32()
        drive_msg.data = self.drive_wheel_position
        
        # consider self.self.invert_drive_motor
        self.drive_motor_pub.publish(drive_msg)
        
        # -------------------------- Pivot Wheel --------------------------------------
        modified_rqst_pivot_angle = max(0, min(6.28, self.rqst_pivot_angle))
        pivot_output = self.pivot_pid_controller.compute_move(modified_rqst_pivot_angle, self.actual_pivot_position)
        pivot_msg = Float32()
        pivot_msg.data = pivot_output
        self.pivot_position_pub.publish(pivot_msg)
        
        # if we are mocking the hardware, then the actual values are the commanded values
        if self.mock_encoder_values:
            self.commanded_wheel_speed = drive_output
            self.commanded_pivot_position = pivot_output
            
        # Say something on the logger
        # if self.commanded_pivot_position != self.actual_pivot_position:
        #     self.get_logger().info(f'{self.module_name}: Pivot Angle: {self.actual_pivot_position}, Requested Pivot Angle: {self.rqst_pivot_angle}, Commanded Pivot Angle: {self.commanded_pivot_position}')

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