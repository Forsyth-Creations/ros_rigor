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