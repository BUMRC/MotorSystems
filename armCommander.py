from typing import List
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64
from rclpy.publisher import Publisher
import board
import busio
import adafruit_pca9685
import time
import adafruit_extended_bus
import warnings
import sys

warnings.filterwarnings('ignore')

#i2c = busio.I2C(board.SCL, board.SDA)
i2c = adafruit_extended_bus.ExtendedI2C(15)
shield = adafruit_pca9685.PCA9685(i2c, address=0x40)
shield.frequency = 250
acceleration = 0 #acceleration speed
delay = 0 #delay to go from 100 to -100

class MotorController(Node):
    def __init__(self, motor_id):
        super().__init__('motor_controller')
        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10)
        self.subscription 
        self.motor_publishers: List[Publisher] = [self.create_publisher(
            Float64,
            f'/motor_{i}/target_velocity',
            10) for i in range(5)]
        self.get_logger().info("MotorController node started")
        
        self.motor_velocities = [0, 0, 0, 0, 0]
        self.motor_id = motor_id
    
    def set_motor(self, motor_index, velocity):
        self.motor_velocities[motor_index] = velocity
        msg = Float64()
        msg.data = velocity
        self.motor_publishers[motor_index].publish(msg)
    
    def cmd_vel_callback(self, msg:Twist):
        linear_speed = msg.linear.x
        angular_speed = msg.angular.z
        time.sleep(0.01)
        self.set_motor(self.motor_id, linear_speed)
            

def main(args=None):
    rclpy.init(args=args)
    motor_id = int(sys.argv[1])
    motor_controller = MotorController(motor_id)
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
