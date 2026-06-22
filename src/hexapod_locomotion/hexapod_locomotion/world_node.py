"""Publie un environnement simple (sol + repères) pour la visualisation RViz.

Un sol plan est publié comme visualization_msgs/MarkerArray sur /environment,
dans le repère odom, afin de ne pas avoir un espace vide sous le robot.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy
from visualization_msgs.msg import Marker, MarkerArray


class WorldNode(Node):

    def __init__(self):
        super().__init__('hexapod_world')
        self.declare_parameter('frame_id', 'odom')
        self.declare_parameter('ground_size', 10.0)
        self.frame = self.get_parameter('frame_id').value
        self.size = self.get_parameter('ground_size').value

        # QoS "transient local" = les nouveaux abonnés (RViz) reçoivent le dernier message.
        qos = QoSProfile(depth=1)
        qos.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
        self.pub = self.create_publisher(MarkerArray, '/environment', qos)
        self.timer = self.create_timer(1.0, self._publish)
        self._publish()
        self.get_logger().info('Environnement (sol) publié sur /environment')

    def _make_ground(self):
        m = Marker()
        m.header.frame_id = self.frame
        m.ns = 'ground'
        m.id = 0
        m.type = Marker.CUBE
        m.action = Marker.ADD
        m.pose.position.z = -0.01     # juste sous z=0 (les pieds touchent z=0)
        m.pose.orientation.w = 1.0
        m.scale.x = self.size
        m.scale.y = self.size
        m.scale.z = 0.02
        m.color.r = 0.18
        m.color.g = 0.22
        m.color.b = 0.18
        m.color.a = 1.0
        return m

    def _publish(self):
        arr = MarkerArray()
        arr.markers.append(self._make_ground())
        self.pub.publish(arr)


def main(args=None):
    rclpy.init(args=args)
    node = WorldNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
