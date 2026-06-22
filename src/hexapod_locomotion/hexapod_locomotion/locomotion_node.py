"""Main ROS2 node for hexapod locomotion.

Subscribes to geometry_msgs/TwistStamped on /cmd_vel and converts
velocity commands into joint angle commands via gait generation + IK.
Publishes sensor_msgs/JointState for visualization and motor control.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import JointState
from std_msgs.msg import Header

from hexapod_locomotion.leg_ik import HexapodModel
from hexapod_locomotion.gait_generator import GaitGenerator, GaitType


JOINT_NAMES = []
for i in range(1, 7):
    JOINT_NAMES.extend([
        f'leg{i}_coxa_joint',
        f'leg{i}_femur_joint',
        f'leg{i}_tibia_joint',
    ])


class LocomotionNode(Node):

    def __init__(self):
        super().__init__('hexapod_locomotion')

        self.declare_parameter('body_radius', 0.07)
        self.declare_parameter('coxa_length', 0.09)
        self.declare_parameter('femur_length', 0.185)
        self.declare_parameter('tibia_length', 0.250)
        self.declare_parameter('body_height', 0.15)
        self.declare_parameter('control_rate', 50.0)
        self.declare_parameter('gait_type', 'tripod')
        self.declare_parameter('step_height', 0.04)
        self.declare_parameter('cycle_period', 1.0)
        self.declare_parameter('max_linear_speed', 0.3)
        self.declare_parameter('max_angular_speed', 1.0)

        self.hexapod = HexapodModel(
            body_radius=self.get_parameter('body_radius').value,
            coxa_length=self.get_parameter('coxa_length').value,
            femur_length=self.get_parameter('femur_length').value,
            tibia_length=self.get_parameter('tibia_length').value,
            body_height=self.get_parameter('body_height').value,
        )

        self.gait = GaitGenerator()
        self.gait.step_height = self.get_parameter('step_height').value
        self.gait.cycle_period = self.get_parameter('cycle_period').value

        gait_name = self.get_parameter('gait_type').value
        try:
            self.gait.set_gait(GaitType(gait_name))
        except ValueError:
            self.get_logger().warn(f'Unknown gait "{gait_name}", defaulting to tripod')
            self.gait.set_gait(GaitType.TRIPOD)

        self.max_lin = self.get_parameter('max_linear_speed').value
        self.max_ang = self.get_parameter('max_angular_speed').value

        self.cmd_vx = 0.0
        self.cmd_vy = 0.0
        self.cmd_omega = 0.0
        self.last_cmd_time = self.get_clock().now()

        self.sub_cmd_vel = self.create_subscription(
            TwistStamped, '/cmd_vel', self._cmd_vel_cb, 10
        )

        self.pub_joint_states = self.create_publisher(
            JointState, '/joint_states', 10
        )

        rate = self.get_parameter('control_rate').value
        self.dt = 1.0 / rate
        self.timer = self.create_timer(self.dt, self._control_loop)

        self.get_logger().info(
            f'Hexapod locomotion node started @ {rate} Hz, gait={gait_name}'
        )

    def _cmd_vel_cb(self, msg: TwistStamped):
        self.cmd_vx = max(-self.max_lin, min(self.max_lin, msg.twist.linear.x))
        self.cmd_vy = max(-self.max_lin, min(self.max_lin, msg.twist.linear.y))
        self.cmd_omega = max(-self.max_ang, min(self.max_ang, msg.twist.angular.z))
        self.last_cmd_time = self.get_clock().now()

    def _control_loop(self):
        elapsed = (self.get_clock().now() - self.last_cmd_time).nanoseconds * 1e-9
        if elapsed > 0.5:
            self.cmd_vx = 0.0
            self.cmd_vy = 0.0
            self.cmd_omega = 0.0

        foot_targets = self.gait.update(
            self.dt,
            self.cmd_vx,
            self.cmd_vy,
            self.cmd_omega,
            self.hexapod.rest_positions,
        )

        try:
            all_angles = self.hexapod.compute_ik_all(foot_targets)
        except Exception as e:
            self.get_logger().error(f'IK failed: {e}')
            all_angles = self.hexapod.get_rest_angles()

        msg = JointState()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = JOINT_NAMES

        positions = []
        for coxa, femur, tibia in all_angles:
            positions.extend([coxa, femur, tibia])
        msg.position = positions
        msg.velocity = [0.0] * 18
        msg.effort = [0.0] * 18

        self.pub_joint_states.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = LocomotionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
