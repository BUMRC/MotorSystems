import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64
import time
from enum import Enum
import numpy as np
from PIDController import PIDController

import adafruit_pca9685
import time
import adafruit_extended_bus


class MotorState(Enum):
    STOPPED = 0
    RUNNING = 1
    ERROR = 2

class Side(Enum):
    LEFT = 0
    RIGHT = 1

class MotorNode(Node):
    def __init__(self, motor_id, config, shield):
        super().__init__(f'motor_node_{motor_id}') # Maybe add side to the name??
        

        # Motor configuration
        self.motor_id = motor_id
        self.side = Side(motor_id % 2)
        self.config = config
        self.shield = shield
        
        # Initialize PID controller
        self.pid = PIDController(
            kp=config['pid']['kp'],
            ki=config['pid']['ki'],
            kd=config['pid']['kd'],
            ff=config['pid']['ff']
        )
        
        # Motor state
        self.state = MotorState.STOPPED
        self.current_velocity = 0.0
        self.target_velocity = 0.0
        self.last_command_time = self.get_clock().now()
        
        # pub sub stuff
        self.velocity_subscriber = self.create_subscription(
            Float64,
            f'/motor_{self.motor_id}/target_velocity',
            self.velocity_callback,
            10
        )
        
        self.status_publisher = self.create_publisher(
            Float64,
            f'/motor_{self.motor_id}/current_velocity',
            10
        )
        
        # loop 50 times per second
        self.control_loop_timer = self.create_timer(
            1.0/config['control_frequency'],
            self.control_loop
        )
        
        self.init_hardware()
        
        self.get_logger().info(f"Motor {motor_id} ({self.side}) initialized")
    
    def init_hardware(self):
        try:
            self.forward_pin = self.shield[self.config['forward_pin_id']]
            self.reverse_pin = self.shield[self.config['reverse_pin_id']]
            self.get_logger().info(f"Forward pin: {self.forward_pin}, Reverse pin: {self.reverse_pin} for motor {self.motor_id}")
        except Exception as e:
            self.get_logger().error(f"Failed to initialize motor hardware: {str(e)}")
            self.state = MotorState.ERROR
    
    def velocity_callback(self, msg: Float64):
        self.target_velocity = msg.data
        self.get_logger().info(f"Target velocity: {self.target_velocity} for motor {self.motor_id}")
        self.last_command_time = self.get_clock().now()
    
    def ramp_velocity(self, current, target):
        max_acceleration = self.config['motor']['max_acceleration']
        dt = 1.0/self.config['control_frequency']
        
        max_velocity_change = max_acceleration * dt
        velocity_error = target - current
        
        if abs(velocity_error) <= max_velocity_change:
            return target
        else:
            return current + np.sign(velocity_error) * max_velocity_change
    
    def zero_minimum_output(self, output):
        # im not sure if this is the best way to do this, but stop any output less than 25%
        minimum_output = self.config['motor']['minimum_output']
        if abs(output) < minimum_output:
            return 0
        return output
    
    def set_motor_output(self, output):
        try:
            # if self.side == Side.RIGHT:
            #     output = -output # reversed since motors are connected backward...
                
            output = self.zero_minimum_output(output)
            output = int(output)
            if output > 0:
                self.get_logger().info(f"Setting motor {self.motor_id} to {output}")
                self.forward_pin.duty_cycle = output
                self.reverse_pin.duty_cycle = 0
            elif output < 0:
                self.get_logger().info(f"Setting motor {self.motor_id} to {output}")
                self.forward_pin.duty_cycle = 0
                self.reverse_pin.duty_cycle = -output
            else:
                # self.get_logger().info(f"Setting motor {self.motor_id} to 0")
                self.forward_pin.duty_cycle = 0
                self.reverse_pin.duty_cycle = 0
            
            self.state = MotorState.RUNNING if output != 0 else MotorState.STOPPED
            
        except Exception as e:
            self.get_logger().error(f"Failed to set motor output: {str(e)}")
            self.state = MotorState.ERROR
    
    def control_loop(self):
        time_since_last_command = (self.get_clock().now() - self.last_command_time).nanoseconds * 1e-9
        if time_since_last_command > self.config['timeout']:
            self.target_velocity = 0.0
            self.get_logger().warn("Command timeout - stopping motor")
        
        ramped_target = self.ramp_velocity(self.current_velocity, self.target_velocity)
        
        output = self.pid.compute(self.current_velocity, ramped_target)
        
        self.set_motor_output(output)
        
        self.current_velocity = ramped_target # TODO: get encoder values
        
        status_msg = Float64()
        status_msg.data = self.current_velocity
        self.status_publisher.publish(status_msg) #TODO: actually set this up for correct data
        
    # on shutdown, set all motors to 0
    def on_shutdown(self):
        self.target_velocity = 0.0
        self.set_motor_output(0)

def main():
    rclpy.init()
    
    i2c = adafruit_extended_bus.ExtendedI2C(15)
    shield = adafruit_pca9685.PCA9685(i2c, address=0x40)
    shield.frequency = 250
        
    motors = []
    for motor_id in range(6):
        config = {
            'pid': {
                'kp': 0.1,
                'ki': 0,
                'kd': 0,
                'ff': 0
            },
            'motor': {
                'max_output': 65535,
                'min_output': -65535,
                'minimum_output': 65535 * 0.25,  # prevent motor damage?
                'max_acceleration': 1000
            },
            'forward_pin_id': motor_id * 2 + (motor_id % 2),
            'reverse_pin_id': motor_id * 2 + 1 - (motor_id % 2),
            'control_frequency': 50, 
            'timeout': 0.5 
        }
        motor_node = MotorNode(motor_id, config, shield)
        motors.append(motor_node)
    
    try:
        rclpy.spin_until_future_complete(motors[0], rclpy.Future())
    except KeyboardInterrupt:
        pass
    finally:
        for motor in motors:
            motor.destroy_node()
        for motor_id in range(6):
            forward_pin = motor_id * 2 + (motor_id % 2)
            reverse_pin = motor_id * 2 + 1 - (motor_id % 2)
            shield[forward_pin].duty_cycle = 0
            shield[reverse_pin].duty_cycle = 0
        rclpy.shutdown()
    
if __name__ == '__main__':
    main()
