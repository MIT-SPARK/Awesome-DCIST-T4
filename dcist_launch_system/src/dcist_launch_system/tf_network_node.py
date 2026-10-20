#!/usr/bin/env python3
"""Republish this robot's TF at a reduced rate for the base station.

Subscribes to tf / tf_static in the node's namespace (/<robot>/tf) and publishes
network/tf / network/tf_static (/<robot>/network/tf), which is the only TF the base
station's tf_relay subscribes to. The full-rate TF therefore never crosses the network.
"""

import rclpy
from rclpy.node import Node
from tf2_msgs.msg import TFMessage

from dcist_launch_system.tf_relay_node import DYNAMIC_QOS, STATIC_QOS


class TfNetwork(Node):
    def __init__(self):
        super().__init__("tf_network")

        self.declare_parameter("rate_hz", 10.0)
        rate_hz = self.get_parameter("rate_hz").get_parameter_value().double_value

        # /tf mixes publishers at different rates, so we throttle per frame (keeping the
        # latest transform for each child frame) instead of dropping whole messages
        self.pending = {}
        self.static_transforms = {}

        self.tf_pub = self.create_publisher(TFMessage, "network/tf", DYNAMIC_QOS)
        self.static_pub = self.create_publisher(
            TFMessage, "network/tf_static", STATIC_QOS
        )
        self.create_subscription(TFMessage, "tf", self.tf_callback, DYNAMIC_QOS)
        self.create_subscription(
            TFMessage, "tf_static", self.static_callback, STATIC_QOS
        )
        self.timer = self.create_timer(1.0 / rate_hz, self.flush)

    def tf_callback(self, msg):
        for tf in msg.transforms:
            self.pending[tf.child_frame_id] = tf

    def flush(self):
        # only send frames that updated, as resending old stamps makes tf2 warn
        if not self.pending:
            return

        self.tf_pub.publish(TFMessage(transforms=list(self.pending.values())))
        self.pending = {}

    def static_callback(self, msg):
        for tf in msg.transforms:
            self.static_transforms[tf.child_frame_id] = tf

        self.static_pub.publish(
            TFMessage(transforms=list(self.static_transforms.values()))
        )


def main(args=None):
    rclpy.init(args=args)

    try:
        node = TfNetwork()
        try:
            rclpy.spin(node)
        finally:
            node.destroy_node()
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()
