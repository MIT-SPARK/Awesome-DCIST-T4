#!/usr/bin/env python3
"""Relay per-robot TF (/<robot>/tf, /<robot>/tf_static) onto this node's namespace.

Run on the base station (namespaced by its own robot name) so that nodes reading
/<base_station>/tf see the frames of every robot.
"""

import re

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from tf2_msgs.msg import TFMessage

TF_TYPE = "tf2_msgs/msg/TFMessage"
TOPIC_REGEX = re.compile(r"^/(?:([^/]+)/)?(tf|tf_static)$")

DYNAMIC_QOS = QoSProfile(depth=100, reliability=ReliabilityPolicy.RELIABLE)
STATIC_QOS = QoSProfile(
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
)


class TfRelay(Node):
    def __init__(self):
        super().__init__("tf_relay")

        # Empty list means relay every /<robot>/tf topic that shows up
        self.declare_parameter("robots", [""])
        self.declare_parameter("relay_global_tf", False)
        self.declare_parameter("discovery_period_s", 1.0)

        robots = self.get_parameter("robots").get_parameter_value().string_array_value
        self.robots = set(x for x in robots if x)
        self.relay_global_tf = (
            self.get_parameter("relay_global_tf").get_parameter_value().bool_value
        )
        period = (
            self.get_parameter("discovery_period_s").get_parameter_value().double_value
        )

        self.tf_pub = self.create_publisher(TFMessage, "tf", DYNAMIC_QOS)
        self.static_pub = self.create_publisher(TFMessage, "tf_static", STATIC_QOS)
        self.output_topics = set(
            [
                self.resolve_topic_name("tf"),
                self.resolve_topic_name("tf_static"),
            ]
        )

        # Static transforms are latched per publisher, so we keep every static
        # transform we've seen and republish the full set (like StaticTransformBroadcaster)
        self.static_transforms = {}
        self.subs = {}

        self.timer = self.create_timer(period, self.discover)
        self.discover()

    def _should_relay(self, topic):
        if topic in self.subs:
            return False

        # guard against remaps pointing an input at our own output
        if self.resolve_topic_name(topic) in self.output_topics:
            return False

        match = TOPIC_REGEX.match(topic)
        if not match:
            return False

        robot = match.group(1)
        if robot is None:
            return self.relay_global_tf

        return not self.robots or robot in self.robots

    def discover(self):
        for topic, types in self.get_topic_names_and_types():
            if TF_TYPE not in types or not self._should_relay(topic):
                continue

            is_static = topic.endswith("tf_static")
            self.get_logger().info(f"Relaying '{topic}'")
            if is_static:
                self.subs[topic] = self.create_subscription(
                    TFMessage, topic, self.static_callback, STATIC_QOS
                )
            else:
                self.subs[topic] = self.create_subscription(
                    TFMessage, topic, self.tf_pub.publish, DYNAMIC_QOS
                )

    def static_callback(self, msg):
        for tf in msg.transforms:
            self.static_transforms[tf.child_frame_id] = tf

        self.static_pub.publish(
            TFMessage(transforms=list(self.static_transforms.values()))
        )


def main(args=None):
    rclpy.init(args=args)

    try:
        node = TfRelay()
        try:
            rclpy.spin(node)
        finally:
            node.destroy_node()
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()
