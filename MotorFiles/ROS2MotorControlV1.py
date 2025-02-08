import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import board
import busio
import adafruit_pca9685
import time


i2c = busio.I2C(board.SCL, board.SDA)
shield = adafruit_pca9685.PCA9685(i2c)
shield.frequency = 250

class MotorController(Node):
    def __init__(self):
        super().__init__('motor_controller')
        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10)
        self.subscription 
        
        self.motors = [(shield.channels[i], shield.channels[i+1]) for i in range(0, 12, 2)]
    
    def set_motor(self, motor_index, direction):
        forward_pin, reverse_pin = self.motors[motor_index]
        if direction == 'forward':
            forward_pin.duty_cycle = 65535
            reverse_pin.duty_cycle = 0
        elif direction == 'backward':
            forward_pin.duty_cycle = 0
            reverse_pin.duty_cycle = 65535
        else:
            forward_pin.duty_cycle = 0
            reverse_pin.duty_cycle = 0
    
    def cmd_vel_callback(self, msg):
        linear_speed = msg.linear.x
        angular_speed = msg.angular.z
        
        if linear_speed > 0:
            self.set_motor(0, 'forward')
            self.set_motor(1, 'forward')
        elif linear_speed < 0:
            self.set_motor(0, 'backward')
            self.set_motor(1, 'backward')
        elif angular_speed > 0:
            self.set_motor(0, 'forward')
            self.set_motor(1, 'backward')
        elif angular_speed < 0:
            self.set_motor(0, 'backward')
            self.set_motor(1, 'forward')
        else:
            self.set_motor(0, 'stop')
            self.set_motor(1, 'stop')

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
            package='your_package',
            executable='motor_controller',
            output='screen'
        ),
        Node(
            package='teleop_twist_keyboard',
            executable='teleop_twist_keyboard',
            output='screen'
        )
    ])

# import rclpy
# from rclpy.node import Node
# from geometry_msgs.msg import Twist

# class MotorControllerTest(Node):
#     def __init__(self):
#         super().__init__('motor_controller_test')
#         self.subscription = self.create_subscription(
#             Twist,
#             '/cmd_vel',
#             self.cmd_vel_callback,
#             10)
#         self.subscription  # Prevent unused variable warning
#         self.get_logger().info("MotorControllerTest node started, waiting for /cmd_vel messages...")

#     def set_motor(self, motor_index, direction):
#         self.get_logger().info(f"Motor {motor_index}: {direction}")

#     def cmd_vel_callback(self, msg):
#         linear_speed = msg.linear.x
#         angular_speed = msg.angular.z

#         if linear_speed > 0:
#             self.set_motor(0, 'forward')
#             self.set_motor(1, 'forward')
#         elif linear_speed < 0:
#             self.set_motor(0, 'backward')
#             self.set_motor(1, 'backward')
#         elif angular_speed > 0:
#             self.set_motor(0, 'forward')
#             self.set_motor(1, 'backward')
#         elif angular_speed < 0:
#             self.set_motor(0, 'backward')
#             self.set_motor(1, 'forward')
#         else:
#             self.set_motor(0, 'stop')
#             self.set_motor(1, 'stop')

# def main(args=None):
#     rclpy.init(args=args)
#     motor_controller_test = MotorControllerTest()
#     rclpy.spin(motor_controller_test)
#     motor_controller_test.destroy_node()
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()
