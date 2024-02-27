import time

class PIDFController:
    def __init__(self, kp, ki, kd, ff):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.ff = ff

        self.setpoint = 0

        self.prev_time = time.time()
                
        self.prev_err = 0
        self.integral = 0
    
    def reset(self):
        """
        Reset the data for the PID controller
        """
        self.prev_time = time.time()
        self.prev_err = 0
        self.integral = 0
        self.setpoint = 0
    
    def clear(self):
        """
        Clear the PID controller settings
        """
        self.kp = 0
        self.ki = 0
        self.kd = 0
        self.kf = 0
        self.reset()
    
    def setTarget(self, setpoint):
        self.setpoint = setpoint
        
    def update(self, value, setpoint = None):
        """
        Update the PID controller with the current value and setpoint
        """
        if setpoint is not None:
            if self.setpoint != setpoint:
                self.reset()
            self.setpoint = setpoint
        # TODO: figure out if time should be provided by the caller?
        dt = time.time() - self.prev_time
        if dt <= 0:
            return 0
        
        
        # error, which is the difference between the setpoint and the current value
        error = self.setpoint - value
        # TODO: figure out when integral should be reset
        self.integral += error * dt
        
        # calculate the pid output (relative)
        proportional = error * self.kp
        integral = self.integral * self.ki
        derivative = (error - self.prev_err) / dt * self.kd
        
        output = proportional + integral + derivative + self.ff
        
        self.prev_err = error
        self.prev_time = time.time()
        return output

        
    