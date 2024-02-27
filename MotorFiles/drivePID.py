#### IMPORTS ####
# Adafruit & PCA board setups
import board
import busio
import adafruit_pca9685
import time
from motor import Motor
import Jetson.GPIO as GPIO

#################

#### SETUP ####
# Connections to board
i2c = busio.I2C(board.SCL, board.SDA)
shield = adafruit_pca9685.PCA9685(i2c)

# Frequency (general)
shield.frequency = 250


################
class DriveTrain:
    def __init__(self):
        a = 0 # placeholder values, replce with encoder pins
        b = 0 # placeholder values, replce with encoder pins
        
        self.motors = [
            Motor(shield, 0, 1, 0, a, b, 'wheel fl'),
            Motor(shield, 2, 3, 1, a, b, 'wheel fr'),
            Motor(shield, 4, 5, 2, a, b, 'wheel ml'),
            Motor(shield, 6, 7, 3, a, b, 'wheel mr'),
            Motor(shield, 8, 9, 4, a, b, 'wheel bl'),
            Motor(shield, 10,11,5, a, b, 'wheel br'),
        ]
    # get a list of all motors   
    def get_motors(self):
        return self.motors
    
    # print all motor ids
    def print_motors(self):
        for motor in self.motors:
            print(motor)

    # set each motor's target velocity to the provided value, uses pid controller to adjust motor speed
    def drive(self, velocity):
        for motor in self.motors:
            motor.set_target_velocity(velocity)
    
    # drive a motor in a direction and speed
    def drive_motor(self, motor_id, velocity):
        self.motors[motor_id].set_target_velocity(velocity)
    
    # stop a motor
    def stop_motor(self, motor_id):
        self.motors[motor_id].stop()
    
    # stop all motors
    def stop(self):
        for motor in self.motors:
            motor.pidController.setTarget(0)
            motor.stop()
    
    # update motor output based on pid controller output, needs to be called periodically
    def update(self):
        for motor in self.motors:
            motor.update()
            
    # open loop controls
    # set a motor direction to forward
    def set_fwd(self, motor_id):
        self.motors[motor_id].set_fwd()
    
    # set all motor directions to forward
    def set_all_fwd(self):
        for motor in self.motors:
            motor.set_fwd()
    
    # set a motor direction to backward
    def set_bwd(self, motor_id):
        self.motors[motor_id].set_bwd()
    
    # set all motor directions to backward
    def set_all_bwd(self):
        for motor in self.motors:
            motor.set_bwd()
    
        # drive a motor in a direction and speed
    def drive_forward(self, speed):
        for motor in self.motors:
            motor.set_fwd()
            motor.set_spd(speed)
    
    # drive a motor in a direction and speed
    def drive_backward(self, speed):
        for motor in self.motors:
            motor.set_bwd()
            motor.set_spd(speed)
    
    def drv_motor(self, motor_id, direction, speed):
        if direction > 0.0:
            self.motors[motor_id].set_fwd()
        else:
            self.motors[motor_id].set_bwd()
        self.motors[motor_id].set_spd(speed)

drivetrain = DriveTrain()
motors = drivetrain.get_motors()
motor_names = {}
for motor in motors:
    motor_names['m'+motor.get_id()] = motor

# TODO: create a thread for the drivetrain to run in, and have the main function be a user input function where the user can control the drivetrain
# for now, the input will only handle one command
def cmd_help(command_list):
    print('\nHere is a list of current possible commands:')
    print()
    print('get_motors : list all motors & locations')
    print('set_fwd MOTOR : set specific MOTOR to forward')
    print('set_bwd MOTOR : set specific MOTOR to backward')
    print('set_all_fwd : set all motors to forward')
    print('set_all_bwd : set all motors to backward')
    print('drv_motor MOTOR DIR SPD : drive MOTOR in DIR direction and SPD speed')
    print('drv : drive all motors in the current direction')
    print('drv_fwd SPD : drive all motors forward at SPD speed')
    print('drv_bwd SPD : drive all motors backward at SPD speed')
    print('p_drive VEL : drive all motors at VEL velocity(Using pid closed loop control) VEL is in rps')
    print('p_drv_motor MOTOR VEL: drive MOTOR at VEL velocity(Using pid closed loop control) VEL is in rps')
    print('stop_motor MOTOR : stop specific MOTOR')
    print('stop : stop all motors')
    print('exit : exit code')
    
    print()


commands = {
    "get_motors": drivetrain.print_motors,
    "set_fwd": drivetrain.set_fwd,
    "set_bwd": drivetrain.set_bwd,
    "set_all_fwd": drivetrain.set_all_fwd,
    "set_all_bwd": drivetrain.set_all_bwd,
    "drv_fwd": drivetrain.drive_forward,
    "drv_bwd": drivetrain.drive_backward,
    "drv_motor": drivetrain.drv_motor,
    "p_drv_motor": drivetrain.drive_motor,
    "p_drive": drivetrain.drive,
    "stop": drivetrain.stop,
    "stop_motor": drivetrain.stop_motor,
    "exit": exit,
    "help": cmd_help,
}

def main():
    while True:
        user_input = input('> ').split()

        if len(user_input) == 0:
            continue
        
        command = user_input[0]
        
        # Invalid CMD [not in command list]
        if command not in commands.keys():
            print('Not a valid command, use \'help\' to see a list of commands')
            continue
        
        # user_input: CMD [stops everything and quits code]
        elif command in ['exit']:
            commands[command]()
            break
        elif command == 'drv_motor':
            if len(user_input) != 4:
                print('Unspecified motor, direction, and/or speed value')
                continue
            MOTOR = user_input[1]
            DIR = user_input[2]
            SPD = user_input[3]
            if MOTOR not in motor_names.keys():
                print('invalid motor name (m0, m1, etc...)')
                continue

            MOTOR = motor_names[user_input[1]]		

            commands[command](MOTOR, float(DIR), float(SPD))
        elif command == "p_drive":
            if len(user_input) != 2:
                print('Unspecified velocity value')
                continue
            VELOCITY = user_input[1]
            drivetrain.drive(float(VELOCITY))
            dt = 0.1
            try:
                while True:
                    drivetrain.update()
                    time.sleep(dt)
            except KeyboardInterrupt:
                drivetrain.stop()
        # user_input: CMD MOTOR DIR SPD
        elif command == 'p_drv_motor':
            if len(user_input) != 3:
                print('Unspecified motor and/or velocity value')
                continue
            MOTOR = user_input[1]
            VELOCITY = user_input[2]
            if MOTOR not in motor_names.keys():
                print('invalid motor name (m0, m1, etc...)')
                continue

            MOTOR = motor_names[MOTOR]		

            drivetrain.drive_motor(MOTOR, float(VELOCITY))
            dt = 0.1
            try:
                while True:
                    drivetrain.update()
                    time.sleep(dt)
            except KeyboardInterrupt:
                drivetrain.stop()
            
        # user_input: CMD MOTOR
        elif command in ['set_fwd', 'set_bwd', 'stop_motor']:
            if len(user_input) != 2:
                print('Unspecified motor, direction, and/or speed value')
                continue
            if user_input[1] not in motor_names.keys():
                print('Invalid motor')
                continue
            MOTOR = motor_names[user_input[1]]

            commands[command](MOTOR)
        
        # user_input: CMD SPD
        elif command in ['drv_fwd', 'drv_bwd']:
            if len(user_input) != 2:
                print('Unspecified speed value')
                continue 
            SPD = user_input[1]
            
            commands[command](float(SPD))

        # user_input CMD [requires commands list]
        elif command in ['help']:
            commands[command](commands)

        # user_input CMD [not requiring any inputs]
        else:
            commands[command]()
    
    GPIO.cleanup()
    drivetrain.stop()

################
try:
    main()
except KeyboardInterrupt:
    GPIO.cleanup()
    drivetrain.stop()
    print('Exiting...')
    exit()