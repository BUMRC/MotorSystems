import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class MotorControlNode(Node):
    def __init__(self):
        super().__init__('motor_control_node')
        self.subscription = self.create_subscription(
            String,
            '/cmd_vel',
            self.cmd_vel_callback,
            10)
        self.motor_speeds = [0] * 6

    def cmd_vel_callback(self, msg):
        command = msg.data.split()
        if command[0] == "drv_motor":
            motor = int(command[1])
            direction = int(command[2])
            speed = int(command[3])
            self.drive_motor(motor, direction, speed)
        elif command[0] == "drv":
            direction = int(command[1])
            speed = int(command[2])
            self.drive_all_motors(direction, speed)
        elif command[0] == "stop_motor":
            motor = int(command[1])
            self.stop_motor(motor)
        elif command[0] == "stop":
            self.stop_all_motors()

    def drive_motor(self, motor, dir, spd):
        # PLACEHOLDER, HOOK UP WITH ACTUAL CONTROLLERS
        self.motor_speeds[motor - 1] = spd

    def drive_all_motors(self, dir, spd):
        # PLACEHOLDER, HOOK UP WITH ACTUAL CONTROLLERS
        self.motor_speeds = [spd] * 6

    def stop_motor(self, motor):
        # PLACEHOLDER, HOOK UP WITH ACTUAL CONTROLLERS
        self.motor_speeds[motor - 1] = 0

    def stop_all_motors(self):
        # PLACEHOLDER, HOOK UP WITH ACTUAL CONTROLLERS
        self.motor_speeds = [0] * 6

    def run(self):
        rclpy.spin(self)


def main(args=None):
    rclpy.init(args=args)
    motor_control_node = MotorControlNode()
    motor_control_node.run()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
