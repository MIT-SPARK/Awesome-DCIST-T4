import rclpy
from rclpy.node import Node
from rclpy.time import Time
from nav_msgs.msg import Odometry
from tf2_ros import Buffer, TransformListener, TransformException


class TfToOdomNode(Node):
    def __init__(self):
        super().__init__('tf_to_odom_node')

        self.declare_parameter('source_frame', 'odom')
        self.declare_parameter('target_frame', 'base_link')
        self.declare_parameter('publish_rate', 10.0)
        self.declare_parameter(
            'pose_covariance_diagonal',
            [1.0e-3, 1.0e-3, 1.0e-3, 1.0e-4, 1.0e-4, 1.0e-4],
        )

        self.source_frame = self.get_parameter('source_frame').value
        self.target_frame = self.get_parameter('target_frame').value
        publish_rate = self.get_parameter('publish_rate').value
        self.cov_diag = self.get_parameter('pose_covariance_diagonal').value

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.odom_pub = self.create_publisher(Odometry, 'tf_odom', 10)
        self.timer = self.create_timer(1.0 / publish_rate, self.timer_callback)

        self.get_logger().info(
            f'Publishing TF ({self.source_frame} -> {self.target_frame}) '
            f'as Odometry at {publish_rate} Hz'
        )

    def timer_callback(self):
        try:
            t = self.tf_buffer.lookup_transform(
                self.source_frame, self.target_frame, Time()
            )
        except TransformException as e:
            self.get_logger().debug(f'TF lookup failed: {e}', throttle_duration_sec=2.0)
            return

        msg = Odometry()
        msg.header.stamp = t.header.stamp
        msg.header.frame_id = self.source_frame
        msg.child_frame_id = self.target_frame

        msg.pose.pose.position.x = t.transform.translation.x
        msg.pose.pose.position.y = t.transform.translation.y
        msg.pose.pose.position.z = t.transform.translation.z
        msg.pose.pose.orientation = t.transform.rotation

        # Set diagonal pose covariance (6x6 row-major, indices 0,7,14,21,28,35)
        for i, idx in enumerate([0, 7, 14, 21, 28, 35]):
            msg.pose.covariance[idx] = self.cov_diag[i]

        # Twist covariance: high values to indicate no velocity info
        for i, idx in enumerate([0, 7, 14, 21, 28, 35]):
            msg.twist.covariance[idx] = 1e6

        self.odom_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = TfToOdomNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
