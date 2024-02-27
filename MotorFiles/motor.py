#### IMPORTS ####
# Adafruit & PCA board setups
import board
import busio
import adafruit_pca9685
import time
import encoder
from PIDFController import PIDFController
import numpy as np

################

class Motor:
    def __init__(self, shield, channel_dir, channel_spd, id, encoder_a, encoder_b, location='None'):
        self.dir = shield.channels[channel_dir]
        self.spd = shield.channels[channel_spd]
        #self.dir = self.ch_dir.duty_cycle
        #self.spd = self.ch_spd.duty_cycle
        
        self.id = id
        self.loc = location
        self.encoder = encoder.Encoder(encoder_a, encoder_b, id, 2786.2, location)
        
        self.pidController = PIDFController(0.1, 0, 0, 0)
        self.output = 0
    
    def get_id(self):
        return self.id
    
    def get_dir(self):
        return self.dir.duty_cycle

    def get_spd(self):
        return self.spd.duty_cycle
    
    def get_encoder_velocity(self):
        return self.encoder.get_velocity()

    def set_fwd(self):
        self.dir.duty_cycle = 65535

    def set_bwd(self):
        self.dir.duty_cycle = 0

    def check_dir(self, new_dir):
        if new_dir != 0:
            new_dir = 65535
        if self.get_dir() != new_dir:
            while self.get_spd() > 10:
                self.set_spd((self.get_spd()/655.35) // 2)
                #time.sleep(0.1)

    def set_dir(self, val): 
        if val > 0.0:
            self.check_dir(1)
            self.set_fwd()
        else:
            self.check_dir(0)
            self.set_bwd()

    def set_spd(self, speed):
        # TODO: ADD IN SMOOTHING FUNCTION! See the check_dir func above for reference, change & add!
        speed = np.clip(speed, 0, 100)
        self.spd.duty_cycle = int(655.35 * speed)
    
    def set_velocity(self, velocity):
        """
        This sets the speed and direction of the motor calculated based on the velocity
        This shold only be used by the PID controller to handle smooth speed/direction changes
        """
        vel = np.clip(vel, -100.0, 100.0)
        # range of vel is -100 to 100 (percent)
        if velocity <= 0:
            self.set_fwd()
        else:
            self.set_bwd()
        self.set_spd(abs(velocity))
    
    def set_target_velocity(self, target_velocity):
        """
        Update the target velocity for the PID controller
        """
        self.pidController.setTarget(target_velocity)
    
    def update(self):
        """
        This should be called periodically to update the motor speed based on the PID controller
        and encoder readings.
        """
        currentVelocity = self.encoder.get_velocity()
        self.output += self.pidController.update(currentVelocity)
        self.set_velocity(self.output)
        
        return currentVelocity
    
    def reset(self):
        self.pidController.reset()
        self.output = 0
    
    def stop(self):
        self.reset()
        self.set_spd(0)
    
    def __str__(self):
        return 'm' + str(self.id) + ' : ' + str(self.loc) + ' : ' + str(self.encoder.get_velocity())
