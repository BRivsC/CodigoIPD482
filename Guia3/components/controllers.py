class PIDController:
    """PID Controller implementation."""
    
    def __init__(self, kp, ki, kd):
        """Initialize PID controller with gains.
        
        Args:
            kp: Proportional gain
            ki: Integral gain
            kd: Derivative gain
        """
        self.kp = kp  # Proportional gain
        self.ki = ki  # Integral gain
        self.kd = kd  # Derivative gain
        self.previous_error = 0
        self.integral = 0
        
    def compute(self, error, dt):
        """Compute control output based on error and time delta.
        
        Args:
            error: Current error
            dt: Time delta since last computation
            
        Returns:
            float: Control output
        """
        # Proportional term
        p_term = self.kp * error
        
        # Integral term (acumula sólo si dt positivo)
        if dt > 0:
            self.integral += error * dt
        i_term = self.ki * self.integral
        
        # Derivative term – evita división por cero
        if dt > 0:
            d_term = self.kd * (error - self.previous_error) / dt
        else:
            d_term = 0.0
        self.previous_error = error
        
        # Controller output
        return p_term + i_term + d_term
    

    def saturate(self, value, min_value, max_value):
        """Saturate the value between min and max values."""
        return max(min_value, min(value, max_value))
    
    def reset(self):
        """Reset controller state."""
        self.previous_error = 0
        self.integral = 0 