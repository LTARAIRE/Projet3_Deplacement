"""Noeud ROS2 principal de locomotion de l'hexapode PhantomX.

Pipeline : geometry_msgs/TwistStamped (/cmd_vel) → démarche tripode en repère
corps → cinématique inverse exacte (phantom_kinematics) → sensor_msgs/JointState.
Publie aussi la TF odom→base_link (intégration de la consigne) pour que le robot
se déplace réellement dans l'espace et repose sur le sol.

Convention de la démarche (sans glissement) : pendant l'appui, le pied se déplace
à l'opposé de la vitesse commandée du corps (rotation incluse) ; pendant le
transfert, il se lève et revient. Le corps avance donc dans le sens commandé.
"""

import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped, TransformStamped
from sensor_msgs.msg import JointState
from tf2_ros import TransformBroadcaster

from hexapod_locomotion import phantom_kinematics as pk


class LocomotionNode(Node):

    def __init__(self):
        super().__init__('hexapod_locomotion')

        self.declare_parameter('control_rate', 30.0)
        self.declare_parameter('cycle_period', 1.0)      # s par cycle de marche
        self.declare_parameter('duty', 0.5)             # fraction d'appui (tripode)
        self.declare_parameter('step_height', 0.04)     # m, hauteur de levée du pied
        self.declare_parameter('max_stride', 0.12)      # m, enjambée max (sécurité)
        self.declare_parameter('cmd_timeout', 0.5)      # s
        self.declare_parameter('lin_deadband', 0.01)    # m/s
        self.declare_parameter('ang_deadband', 0.03)    # rad/s
        self.declare_parameter('base_height', pk.BASE_HEIGHT)
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')

        self.cycle = self.get_parameter('cycle_period').value
        self.duty = self.get_parameter('duty').value
        self.step_h = self.get_parameter('step_height').value
        self.max_stride = self.get_parameter('max_stride').value
        self.cmd_timeout = self.get_parameter('cmd_timeout').value
        self.lin_db = self.get_parameter('lin_deadband').value
        self.ang_db = self.get_parameter('ang_deadband').value
        self.base_height = self.get_parameter('base_height').value
        self.odom_frame = self.get_parameter('odom_frame').value
        self.base_frame = self.get_parameter('base_frame').value

        self.cmd_vx = self.cmd_vy = self.cmd_omega = 0.0
        self.last_cmd_time = self.get_clock().now()
        self.last_loop_time = self.get_clock().now()
        self.phase = 0.0
        # Pose d'odométrie intégrée
        self.odom_x = self.odom_y = self.odom_yaw = 0.0
        # Mémoire des angles pour le warm-start de l'IK (continuité)
        self.q = [list(pk.REST_Q) for _ in range(6)]

        self.sub_cmd_vel = self.create_subscription(
            TwistStamped, '/cmd_vel', self._cmd_vel_cb, 10)
        self.pub_joint_states = self.create_publisher(JointState, '/joint_states', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        rate = self.get_parameter('control_rate').value
        self.dt = 1.0 / rate
        self.timer = self.create_timer(self.dt, self._control_loop)

        self.get_logger().info(
            f'Noeud locomotion PhantomX démarré @ {rate:.0f} Hz '
            f'(FK/IK exacte, cycle {self.cycle}s)')

    def _cmd_vel_cb(self, msg: TwistStamped):
        self.cmd_vx = msg.twist.linear.x
        self.cmd_vy = msg.twist.linear.y
        self.cmd_omega = msg.twist.angular.z
        self.last_cmd_time = self.get_clock().now()

    def _control_loop(self):
        now = self.get_clock().now()
        # dt réel mesuré (robuste si la boucle ne tient pas la cadence nominale)
        dt = (now - self.last_loop_time).nanoseconds * 1e-9
        self.last_loop_time = now
        dt = min(max(dt, 1e-4), 0.2)   # garde-fous

        elapsed = (now - self.last_cmd_time).nanoseconds * 1e-9
        if elapsed > self.cmd_timeout:
            self.cmd_vx = self.cmd_vy = self.cmd_omega = 0.0

        vx, vy, omega = self.cmd_vx, self.cmd_vy, self.cmd_omega
        moving = (math.hypot(vx, vy) > self.lin_db) or (abs(omega) > self.ang_db)

        # Avance de la phase + intégration de l'odométrie quand le robot bouge
        if moving:
            self.phase = (self.phase + dt / self.cycle) % 1.0
            self.odom_yaw += omega * dt
            cy, sy = math.cos(self.odom_yaw), math.sin(self.odom_yaw)
            self.odom_x += (vx * cy - vy * sy) * dt
            self.odom_y += (vx * sy + vy * cy) * dt

        names, positions = [], []
        for i in range(6):
            rx, ry, rz = pk.REST_FEET[i]
            # Vitesse du pied au sol = - (vitesse corps + rotation au point d'appui)
            vfx, vfy = pk.foot_ground_velocity(rx, ry, vx, vy, omega)
            # Limite l'enjambée par sécurité (évite les cibles inatteignables)
            stride = math.hypot(vfx, vfy) * self.duty * self.cycle
            if stride > self.max_stride and stride > 1e-9:
                scale = self.max_stride / stride
                vfx *= scale
                vfy *= scale

            leg_phase = (self.phase + 0.5 * (i % 2)) % 1.0
            if moving:
                dx, dy, dz = pk.gait_offset(leg_phase, vfx, vfy,
                                            self.duty, self.cycle, self.step_h)
            else:
                dx, dy, dz = 0.0, 0.0, 0.0

            target = (rx + dx, ry + dy, rz + dz)
            q = pk.ik(i, target, q_seed=self.q[i])
            self.q[i] = list(q)

            j_c1, j_thigh, j_tibia = pk.LEG_JOINT_NAMES[i]
            names.extend([j_c1, j_thigh, j_tibia])
            positions.extend([float(q[0]), float(q[1]), float(q[2])])

        now = self.get_clock().now().to_msg()

        js = JointState()
        js.header.stamp = now
        js.name = names
        js.position = positions
        self.pub_joint_states.publish(js)

        tf = TransformStamped()
        tf.header.stamp = now
        tf.header.frame_id = self.odom_frame
        tf.child_frame_id = self.base_frame
        tf.transform.translation.x = self.odom_x
        tf.transform.translation.y = self.odom_y
        tf.transform.translation.z = self.base_height
        tf.transform.rotation.z = math.sin(self.odom_yaw / 2.0)
        tf.transform.rotation.w = math.cos(self.odom_yaw / 2.0)
        self.tf_broadcaster.sendTransform(tf)


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
