import numpy as np

class SteeringController():
    def __init__(self, frame_width=320, frame_height=240, lookahead_distance=0.7):
        """
        Initialize the steering controller.
        
        Parameters:
        - frame_width: Width of the frame in pixels
        - frame_height: Height of the frame in pixels
        - lookahead_distance: How far ahead to look (0.0 to 1.0, where 1.0 is top of frame)
        """
        self.frame_width = frame_width
        self.frame_height = frame_height
        self.lookahead_distance = lookahead_distance
        self.center_x = frame_width // 2
        
        # PID controller parameters
        self.kp = 0.5  # Proportional gain
        self.ki = 0.0  # Integral gain (optional, set to 0 for now)
        self.kd = 0.1  # Derivative gain
        
        # State variables
        self.prev_error = 0
        self.integral = 0
        
    def calculate_steering_angle(self, llane, rlane):
        """
        Calculate steering angle based on detected lane positions.
        
        Parameters:
        - llane: Tuple of (x_coords, y_coords) for left lane
        - rlane: Tuple of (x_coords, y_coords) for right lane
        
        Returns:
        - steering_angle: Angle in degrees (-90 to +90, negative = left, positive = right)
        - lane_center: Calculated center of the lane
        """
        lx, ly = llane
        rx, ry = rlane
        
        # If no lane detected, return 0 (straight)
        if len(lx) == 0 and len(rx) == 0:
            return 0.0, self.center_x
        
        # Calculate lookahead y position
        lookahead_y = int(self.frame_height * (1 - self.lookahead_distance))
        
        # Find lane centers at lookahead point
        left_x = self._get_x_at_y(lx, ly, lookahead_y)
        right_x = self._get_x_at_y(rx, ry, lookahead_y)
        
        # Calculate lane center
        if left_x is not None and right_x is not None:
            lane_center = (left_x + right_x) / 2
        elif left_x is not None:
            # Only left lane detected, estimate center
            lane_center = left_x + 50  # Assume lane width ~100 pixels
        elif right_x is not None:
            # Only right lane detected, estimate center
            lane_center = right_x - 50
        else:
            # Use bottom points if lookahead fails
            if len(lx) > 0 and len(rx) > 0:
                lane_center = (lx[-1] + rx[-1]) / 2
            elif len(lx) > 0:
                lane_center = lx[-1] + 50
            elif len(rx) > 0:
                lane_center = rx[-1] - 50
            else:
                return 0.0, self.center_x
        
        # Calculate error (deviation from center)
        error = lane_center - self.center_x
        
        # PID control
        self.integral += error
        derivative = error - self.prev_error
        
        # Calculate control output
        output = (self.kp * error + 
                 self.ki * self.integral + 
                 self.kd * derivative)
        
        self.prev_error = error
        
        # Convert to steering angle (normalize and scale)
        # Max deviation is roughly half frame width
        max_deviation = self.frame_width / 2
        normalized_output = np.clip(output / max_deviation, -1.0, 1.0)
        
        # Scale to degrees (-45 to +45 is reasonable for most applications)
        steering_angle = normalized_output * 45.0
        
        return steering_angle, lane_center
    
    def _get_x_at_y(self, x_coords, y_coords, target_y):
        """
        Get x coordinate at a specific y coordinate using interpolation.
        
        Parameters:
        - x_coords: List of x coordinates
        - y_coords: List of y coordinates
        - target_y: Target y coordinate
        
        Returns:
        - x coordinate at target_y, or None if not available
        """
        if len(x_coords) == 0 or len(y_coords) == 0:
            return None
        
        x_coords = np.array(x_coords)
        y_coords = np.array(y_coords)
        
        # Check if target_y is within range
        if target_y < y_coords.min() or target_y > y_coords.max():
            # If out of range, use closest point
            if target_y < y_coords.min():
                idx = np.argmin(y_coords)
            else:
                idx = np.argmax(y_coords)
            return x_coords[idx]
        
        # Interpolate
        return np.interp(target_y, y_coords, x_coords)
    
    def reset(self):
        """Reset the controller state."""
        self.prev_error = 0
        self.integral = 0
    
    def set_gains(self, kp=None, ki=None, kd=None):
        """Update PID gains."""
        if kp is not None:
            self.kp = kp
        if ki is not None:
            self.ki = ki
        if kd is not None:
            self.kd = kd
    
    def set_lookahead(self, distance):
        """
        Update lookahead distance.
        
        Parameters:
        - distance: Value between 0.0 (bottom) and 1.0 (top)
        """
        self.lookahead_distance = np.clip(distance, 0.0, 1.0)