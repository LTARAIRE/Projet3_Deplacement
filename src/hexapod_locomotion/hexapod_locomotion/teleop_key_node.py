"""Keyboard teleoperation node for hexapod.

Reads keyboard input and publishes geometry_msgs/TwistStamped on /cmd_vel.

Controls:
    z/s  : forward / backward
    q/d  : strafe left / right
    a/e  : rotate left / right
    space: stop
    1/2/3: switch gait (tripod/wave/ripple)
    +/-  : increase/decrease speed
    Ctrl+C: quit
"""

import sys
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped

if sys.platform == 'win32':
    import msvcrt

    def get_key():
        if msvcrt.kbhit():
            return msvcrt.getch().decode('utf-8', errors='ignore')
        return None
else:
    import termios
    import tty
    import select

    _settings = None

    def _save_terminal():
        global _settings
        _settings = termios.tcgetattr(sys.stdin)

    def _restore_terminal():
        if _settings:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, _settings)

    def get_key():
        _save_terminal()
        try:
            tty.setraw(sys.stdin.fileno())
            rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
            if rlist:
                return sys.stdin.read(1)
            return None
        finally:
            _restore_terminal()


HELP_TEXT = """
Hexapod Teleop - Keyboard Control
----------------------------------
  z/s  : forward / backward
  q/d  : strafe left / right
  a/e  : rotate left / right
  space: stop
  1/2/3: gait tripod/wave/ripple
  +/-  : speed up / slow down
  Ctrl+C: quit
"""


class TeleopKeyNode(Node):

    def __init__(self):
        super().__init__('hexapod_teleop_key')

        self.pub = self.create_publisher(TwistStamped, '/cmd_vel', 10)

        self.speed = 0.1
        self.angular_speed = 0.5
        self.speed_step = 0.05

        self.vx = 0.0
        self.vy = 0.0
        self.omega = 0.0

        self.timer = self.create_timer(0.1, self._tick)

        self.get_logger().info(HELP_TEXT)

    def _tick(self):
        key = get_key()

        if key is not None:
            if key == 'z':
                self.vx = self.speed
                self.vy = 0.0
                self.omega = 0.0
            elif key == 's':
                self.vx = -self.speed
                self.vy = 0.0
                self.omega = 0.0
            elif key == 'q':
                self.vx = 0.0
                self.vy = self.speed
                self.omega = 0.0
            elif key == 'd':
                self.vx = 0.0
                self.vy = -self.speed
                self.omega = 0.0
            elif key == 'a':
                self.vx = 0.0
                self.vy = 0.0
                self.omega = self.angular_speed
            elif key == 'e':
                self.vx = 0.0
                self.vy = 0.0
                self.omega = -self.angular_speed
            elif key == ' ':
                self.vx = 0.0
                self.vy = 0.0
                self.omega = 0.0
            elif key == '+':
                self.speed = min(1.0, self.speed + self.speed_step)
                self.get_logger().info(f'Speed: {self.speed:.2f} m/s')
            elif key == '-':
                self.speed = max(0.05, self.speed - self.speed_step)
                self.get_logger().info(f'Speed: {self.speed:.2f} m/s')

        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        msg.twist.linear.x = self.vx
        msg.twist.linear.y = self.vy
        msg.twist.angular.z = self.omega

        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = TeleopKeyNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
