import numpy as np
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster


def quat_conjugate(q):
    """Return conjugate of quaternion [x, y, z, w]."""
    return np.array([-q[0], -q[1], -q[2], q[3]])


def quat_rotate(q, v):
    """Rotate vector v by quaternion q [x, y, z, w]."""
    # q * (0, v) * q_conj
    qv = np.array([
        q[3] * v[0] + q[1] * v[2] - q[2] * v[1],
        q[3] * v[1] + q[2] * v[0] - q[0] * v[2],
        q[3] * v[2] + q[0] * v[1] - q[1] * v[0],
    ])
    # This is the simplified Hamilton product for pure quaternion rotation
    w = -(q[0] * v[0] + q[1] * v[1] + q[2] * v[2])
    return np.array([
        -w * q[0] + qv[0] * q[3] - qv[1] * q[2] + qv[2] * q[1],
        -w * q[1] + qv[1] * q[3] - qv[2] * q[0] + qv[0] * q[2],
        -w * q[2] + qv[2] * q[3] - qv[0] * q[1] + qv[1] * q[0],
    ])


class OdomToTfNode(Node):
    def __init__(self):
        super().__init__('odom_to_tf_node')

        self.declare_parameter('odom_topic', 'gps_fused_odometry/odom')
        self.declare_parameter('parent_frame', 'base_link')
        self.declare_parameter('child_frame', 'world_utm')
        self.declare_parameter('invert', False)

        odom_topic = self.get_parameter('odom_topic').value
        self.parent_frame = self.get_parameter('parent_frame').value
        self.child_frame = self.get_parameter('child_frame').value
        self.invert = self.get_parameter('invert').value

        self.tf_broadcaster = TransformBroadcaster(self)
        self.sub = self.create_subscription(Odometry, odom_topic, self.odom_callback, 10)

        mode = 'inverted' if self.invert else 'direct'
        self.get_logger().info(
            f'Publishing {mode} TF ({self.parent_frame} -> {self.child_frame}) '
            f'from topic {odom_topic}'
        )

    def odom_callback(self, msg: Odometry):
        t = TransformStamped()
        t.header.stamp = msg.header.stamp
        t.header.frame_id = self.parent_frame
        t.child_frame_id = self.child_frame

        pos = msg.pose.pose.position
        ori = msg.pose.pose.orientation

        if self.invert:
            q = np.array([ori.x, ori.y, ori.z, ori.w])
            translation = np.array([pos.x, pos.y, pos.z])

            q_inv = quat_conjugate(q)
            t_inv = -quat_rotate(q_inv, translation)

            t.transform.translation.x = t_inv[0]
            t.transform.translation.y = t_inv[1]
            t.transform.translation.z = t_inv[2]
            t.transform.rotation.x = q_inv[0]
            t.transform.rotation.y = q_inv[1]
            t.transform.rotation.z = q_inv[2]
            t.transform.rotation.w = q_inv[3]
        else:
            t.transform.translation.x = pos.x
            t.transform.translation.y = pos.y
            t.transform.translation.z = pos.z
            t.transform.rotation = ori

        self.tf_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = OdomToTfNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
