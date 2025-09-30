import time

class PID:
    def __init__(self, Kp, Ki, Kd, output_limits=(None, None)):
        self.Kp, self.Ki, self.Kd = Kp, Ki, Kd
        self.min_out, self.max_out = output_limits
        self.last_error = 0.0
        self.integral = 0.0
        self.last_time = None

    def update(self, error, dt=None):
        now = time.time()
        if dt is None:
            dt = now - self.last_time if self.last_time else 0.0
        self.last_time = now

        self.integral += error * dt
        derivative = (error - self.last_error) / dt if dt > 0 else 0.0 
        output = self.Kp*error + self.Ki*self.integral + self.Kd*derivative

        if self.min_out is not None:
            output = max(self.min_out, output)
        if self.max_out is not None:
            output = min(self.max_out, output)
        
        self.last_error = error
        return output