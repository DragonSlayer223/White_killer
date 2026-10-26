class BallBalancerPID:
    def __init__(self):
        # PID Gains (tune these experimentally on your hardware)
        self.Kp = 0.2
        self.Ki = 0.0
        self.Kd = 0.4  # Derivative term is critical for damping!
        
        self.prev_error_x = 0.0
        self.prev_error_y = 0.0
        self.integral_x = 0.0
        self.integral_y = 0.0
        
        # Physical safety limits for plate tilt (e.g., max 12 degrees)
        self.max_tilt_deg = 12.0

    def reset(self, target_x, target_y, current_x, current_y):
        """Reset PID history to the current ball position to avoid a derivative kick."""
        self.prev_error_x = current_x - target_x
        self.prev_error_y = current_y - target_y
        self.integral_x = 0.0
        self.integral_y = 0.0

    def compute(self, target_x, target_y, current_x, current_y, dt):
        # 1. Calculate errors
        error_x = current_x - target_x
        error_y = current_y - target_y
        
        # 2. Integral terms (with anti-windup clamping)
        self.integral_x += error_x * dt
        self.integral_y += error_y * dt
        
        # 3. Derivative terms (rate of change of error)
        derivative_x = (error_x - self.prev_error_x) / dt if dt > 0 else 0.0
        derivative_y = (error_y - self.prev_error_y) / dt if dt > 0 else 0.0
        
        # 4. PID Output -> Map to Plate Tilts
        # Note the negative sign: positive error means tilt the opposite way to correct
        pitch = -(self.Kp * error_y + self.Ki * self.integral_y + self.Kd * derivative_y)
        roll  =   self.Kp * error_x + self.Ki * self.integral_x + self.Kd * derivative_x
        
        # 5. Clamp to safe hardware tilt limits
        pitch = max(-self.max_tilt_deg, min(self.max_tilt_deg, pitch))
        roll  = max(-self.max_tilt_deg, min(self.max_tilt_deg, roll))
        
        # Save state for next iteration
        self.prev_error_x = error_x
        self.prev_error_y = error_y
        
        return pitch, roll
