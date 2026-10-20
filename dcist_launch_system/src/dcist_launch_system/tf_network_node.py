#!/usr/bin/env python3
"""Republish this robot's TF at a reduced rate for the base station.

Subscribes to tf / tf_static in the node's namespace (/<robot>/tf) and publishes
network/tf / network/tf_static (/<robot>/network/tf), which is the only TF the base
station's tf_relay subscribes to. The full-rate TF therefore never crosses the network.
"""

import time

import rclpy
from rclpy.node import Node
from ros_system_monitor_msgs.msg import NodeInfoMsg
from tf2_msgs.msg import TFMessage

from dcist_launch_system.tf_relay_node import DYNAMIC_QOS, STATIC_QOS, STATIC_SUB_QOS


class TfNetwork(Node):
    def __init__(self):
        super().__init__("tf_network")

        self.declare_parameter("rate_hz", 10.0)
        self.declare_parameter("stale_after_s", 2.0)
        rate_hz = self.get_parameter("rate_hz").get_parameter_value().double_value
        self.stale_after_s = (
            self.get_parameter("stale_after_s").get_parameter_value().double_value
        )

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
            TFMessage, "tf_static", self.static_callback, STATIC_SUB_QOS
        )
        self.timer = self.create_timer(1.0 / rate_hz, self.flush)

        # TF stats for the status monitor. Uses the steady clock so sim time and paused
        # bags don't affect staleness
        self.frames = set()
        self.last_msg_time = None
        self.n_in = 0
        self.n_out = 0
        self.last_hb_time = time.monotonic()

        self.heartbeat_pub = self.create_publisher(NodeInfoMsg, "~/node_status", 1)
        self.hb_timer = self.create_timer(1.0, self.hb_callback)

    def tf_callback(self, msg):
        self.last_msg_time = time.monotonic()
        self.n_in += 1
        for tf in msg.transforms:
            self.pending[tf.child_frame_id] = tf
            self.frames.add(tf.child_frame_id)

    def flush(self):
        # only send frames that updated, as resending old stamps makes tf2 warn
        if not self.pending:
            return

        self.tf_pub.publish(TFMessage(transforms=list(self.pending.values())))
        self.pending = {}
        self.n_out += 1

    def static_callback(self, msg):
        for tf in msg.transforms:
            self.static_transforms[tf.child_frame_id] = tf

        self.static_pub.publish(
            TFMessage(transforms=list(self.static_transforms.values()))
        )

    def hb_callback(self):
        t_now = time.monotonic()
        dt = t_now - self.last_hb_time
        self.last_hb_time = t_now
        rate_in = self.n_in / dt
        rate_out = self.n_out / dt
        self.n_in = 0
        self.n_out = 0

        input_topic = self.resolve_topic_name("tf")
        if self.last_msg_time is None:
            status = NodeInfoMsg.WARNING
            notes = f"No TF received on {input_topic}"
        elif t_now - self.last_msg_time > self.stale_after_s:
            status = NodeInfoMsg.WARNING
            age = t_now - self.last_msg_time
            notes = f"No TF on {input_topic} for {age:.0f} s"
        else:
            status = NodeInfoMsg.NOMINAL
            # a subscriber means the base station relay is connected
            n_subs = self.tf_pub.get_subscription_count()
            connected = "base station connected" if n_subs else "no subscribers"
            notes = (
                f"{len(self.frames)} frames, {rate_in:.0f} Hz in, "
                f"{rate_out:.1f} Hz out ({connected})"
            )

        msg = NodeInfoMsg()
        msg.nickname = "tf_network"
        msg.node_name = self.get_fully_qualified_name()
        msg.status = status
        msg.notes = notes
        self.heartbeat_pub.publish(msg)


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
