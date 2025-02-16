import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64
import time
import warnings

warnings.filterwarnings('ignore')

class MotorController(Node):
    def __init__(self):
        super().__init__('motor_controller')
        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10)
        self.subscription 
        self.motor_publishers = [self.create_publisher(
            Float64,
            f'/motor_{i}/target_velocity',
            10) for i in range(6)]
        self.get_logger().info("MotorController node started")
        
        self.motor_velocities = [0, 0, 0, 0, 0, 0]
    
    def set_motor(self, motor_index, velocity):
        self.motor_velocities[motor_index] = velocity
        msg = Float64()
        msg.data = velocity
        self.motor_publishers[motor_index].publish(msg)
        self.get_logger().info(f"Motor {motor_index} set to {velocity}")
    
    def cmd_vel_callback(self, msg: Twist):
        linear_speed = msg.linear.x
        angular_speed = msg.angular.z
        print(f"linear_speed: {linear_speed}, angular_speed: {angular_speed}")
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
            self.set_motor(2, 65535)
            self.set_motor(3, -65535)
            self.set_motor(4, 65535)
            self.set_motor(5, -65535)
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
