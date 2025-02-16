import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import board
import busio
import adafruit_pca9685
import time
import adafruit_extended_bus
import warnings

warnings.filterwarnings('ignore')

#i2c = busio.I2C(board.SCL, board.SDA)
i2c = adafruit_extended_bus.ExtendedI2C(15)
shield = adafruit_pca9685.PCA9685(i2c, address=0x40)
shield.frequency = 250
acceleration = 0 #acceleration speed
delay = 0 #delay to go from 100 to -100

class MotorController(Node):
    def __init__(self):
        super().__init__('motor_controller')
        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10)
        self.subscription 
        self.get_logger().info("MotorController node started")
        self.currentDirection = [0, 0, 0, 0, 0, 0]
        self.motors = [(shield.channels[i], shield.channels[i+1]) for i in range(0, 12, 2)]
    
    def set_motor(self, motor_index, velocity):
        forward_pin, reverse_pin = self.motors[motor_index]
        if abs(velocity) < 1000:
            velocity = 0
        if motor_index % 2 == 1:
            print(f"Swap direction on motor {motor_index}")
            velocity *= -1
        

        if velocity != 0:
            velocity = self.currentDirection[motor_index] + (velocity - self.currentDirection[motor_index]) * 0.01
        self.currentDirection[motor_index] = velocity

        print(f"Motor vel: {velocity} for {motor_index}")
        if abs(velocity) < 32767:
            velocity = 0
        
        if velocity > 0:
            forward_pin.duty_cycle = max(32767,min(int(velocity), 65535))
            reverse_pin.duty_cycle = 0

        elif velocity < 0:
            forward_pin.duty_cycle = 0
            reverse_pin.duty_cycle = max(32767,min(int(-velocity), 65535))

        else:
            forward_pin.duty_cycle = 0
            reverse_pin.duty_cycle = 0

    
    def cmd_vel_callback(self, msg):
        linear_speed = msg.linear.x
        angular_speed = msg.angular.z
        time.sleep(0.01)
        if linear_speed > 0:
            self.get_logger().info("f")
            self.set_motor(0, 65535)
            self.set_motor(1, 65535)
            self.set_motor(2, 65535)
            self.set_motor(3, 65535)
            self.set_motor(4, 65535)
            self.set_motor(5, 65535)
        elif linear_speed < 0:
            self.get_logger().info("b")
            self.set_motor(0, -65535)
            self.set_motor(1, -65535)
            self.set_motor(2, -65535)
            self.set_motor(3, -65535)
            self.set_motor(4, -65535)
            self.set_motor(5, -65535)
        elif angular_speed > 0:
            self.get_logger().info("cw")
            self.set_motor(0, 65535)
            self.set_motor(1, -65535)
            self.set_motor(2, -65535)
            self.set_motor(3, 65535)
            self.set_motor(4, -65535)
            self.set_motor(5, 65535)

        elif angular_speed < 0:
            self.get_logger().info("ccw")
            self.set_motor(0, -65535)
            self.set_motor(1, 65535)
            self.set_motor(2, -65535)
            self.set_motor(3, 65535)
            self.set_motor(4, -65535)
            self.set_motor(5, 65535)
        else:
            self.get_logger().info("stop")
            self.set_motor(0, 0)
            self.set_motor(1, 0)
            self.set_motor(2, 0)
            self.set_motor(3, 0)
            self.set_motor(4, 0)
            self.set_motor(5, 0)
            

def main(args=None):
    rclpy.init(args=args)
    motor_controller = MotorController()
    rclpy.spin(motor_controller)
    motor_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='motor_controls',
            executable='motor_controller',
            output='screen'
        ),
        Node(
            package='teleop_twist_keyboard',
            executable='teleop_twist_keyboard',
            output='screen'
        )
    ])