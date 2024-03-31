import rclpy
from rclpy.node import Node
from rover_msgs.msg import MotorCommand


class CommandPublisher(Node):
    def __init__(self):
        super().__init__('command_publisher')
        self.publisher_ = self.create_publisher(MotorCommand, '/cmd_vel', 10)
        self.timer = self.create_timer(1, self.publish_command)
        self.command_count = 0

    def publish_command(self, motor, dir, spd):
        command_msg = MotorCommand()
        command_msg.motor = motor
        command_msg.direction = dir
        command_msg.speed = spd
        command_msg.command_type = MotorCommand.DRV_MOTOR
        self.publisher_.publish(command_msg)
        self.command_count += 1


def main(args=None):
    rclpy.init(args=args)
    command_publisher = CommandPublisher()
    rclpy.spin(command_publisher)
    command_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
