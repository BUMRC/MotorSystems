class PIDController:
    def __init__(self, kp, ki, kd, ff):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.ff = ff
        
        self.previous_error = 0
        self.integral = 0
        
    def compute(self, current_value, setpoint, dt):
        error = setpoint - current_value

        p_term = self.kp * error
        self.integral += error * dt
        
        i_term = self.ki * self.integral
        d_term = self.kd * (error - self.previous_error) / dt if dt > 0 else 0
        self.previous_error = error

        output = p_term + i_term + d_term
                
        return current_value + output

    
    def reset(self):
        self.integral = 0
        self.previous_error = 0

def main():
    pid = PIDController(kp=0.1, ki=0.01, kd=0.01)
    current_value = 0
    setpoint = 10
    dt = 0.1
    for i in range(100):
        current_value = pid.compute(current_value, setpoint, dt)
        print(f"Current value: {current_value}")

if __name__ == "__main__":
    main()
