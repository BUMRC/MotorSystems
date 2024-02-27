import Jetson.GPIO as GPIO
import time

# Encoder setup
class Encoder:
    def __init__(self, channel_a, channel_b, motor_id, ppr, location='None'):
        self.id = motor_id
        self.loc = location
        self.ppr = ppr
        
        # Set up encoder channels
        self.ch_a = channel_a
        self.ch_b = channel_b
        
        # Initialize GPIO setup
        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(self.ch_a, GPIO.IN)
        GPIO.setup(self.ch_b, GPIO.IN)
        
        # Initialize variables for tracking rotation
        self.last_state_A = GPIO.input(self.ch_a)
        self.pulse_count = 0
        self.prev_time = time.time()
        self.prev_vel = 0
        
        # Add event detection for encoder channel A
        GPIO.add_event_detect(self.ch_a, GPIO.BOTH, callback=self.encoder_callback)

    def encoder_callback(self, channel):
        """
        Callback function for encoder channel A
        
        This function is called whenever the encoder channel A changes state.
        It is used to determine the direction of rotation and update the pulse count.
        """
        current_state_A = GPIO.input(self.ch_a)
        current_state_B = GPIO.input(self.ch_b)
        if current_state_A != self.last_state_A:
            if current_state_B != current_state_A:  
                self.pulse_count += 1  # Clockwise (increment) 

            else:
                self.pulse_count -= 1  # Counter-clockwise (decrement)

    def get_velocity(self):
        """
        Calculate the velocity of the motor based on the pulse count and time since last update
        """
        self.dt = time.time() - self.prev_time
        if self.dt <= 0:
            return self.prev_vel
        
        self.prev_vel = (self.pulse_count / self.ppr) / self.dt
        
        self.prev_time = time.time()
        self.pulse_count = 0
        return self.prev_vel # revolations per second