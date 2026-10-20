#!/usr/bin/env python3
"""Relay every robot's network TF (/<robot>/network/tf) onto this node's namespace.

Run on the base station (namespaced by its own robot name) so that nodes reading
/<base_station>/tf see the frames of every robot. Robots publish the reduced-rate
/<robot>/network/tf with tf_network_node.
"""

import re
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from ros_system_monitor_msgs.msg import NodeInfoMsg
from tf2_msgs.msg import TFMessage

TF_TYPE = "tf2_msgs/msg/TFMessage"
TOPIC_REGEX = re.compile(r"^/([^/]+)/network/(tf|tf_static)$")

DYNAMIC_QOS = QoSProfile(depth=100, reliability=ReliabilityPolicy.RELIABLE)
STATIC_QOS = QoSProfile(
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
)


class TfRelay(Node):
    def __init__(self):
        super().__init__("tf_relay")

        # Empty list means relay every /<robot>/network/tf topic that shows up
        self.declare_parameter("robots", [""])
        self.declare_parameter("discovery_period_s", 1.0)
        self.declare_parameter("stale_after_s", 2.0)

        robots = self.get_parameter("robots").get_parameter_value().string_array_value
        self.robots = set(x for x in robots if x)
        self.own_name = self.get_namespace().strip("/")
        period = (
            self.get_parameter("discovery_period_s").get_parameter_value().double_value
        )
        self.stale_after_s = (
            self.get_parameter("stale_after_s").get_parameter_value().double_value
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

        # Per-robot TF stats for the status monitor. Uses the steady clock so sim time
        # and paused bags don't affect staleness
        self.last_msg_time = {}
        self.msg_counts = {}
        self.last_hb_time = time.monotonic()

        self.heartbeat_pub = self.create_publisher(NodeInfoMsg, "~/node_status", 1)
        self.hb_timer = self.create_timer(1.0, self.hb_callback)

        self.timer = self.create_timer(period, self.discover)
        self.discover()

    def _get_robot(self, topic):
        if topic in self.subs:
            return None

        # guard against remaps pointing an input at our own output
        if self.resolve_topic_name(topic) in self.output_topics:
            return None

        match = TOPIC_REGEX.match(topic)
        if not match:
            return None

        robot = match.group(1)
        if robot == self.own_name:
            return None

        if self.robots and robot not in self.robots:
            return None

        return robot

    def discover(self):
        for topic, types in self.get_topic_names_and_types():
            if TF_TYPE not in types:
                continue

            robot = self._get_robot(topic)
            if robot is None:
                continue

            self.get_logger().info(f"Relaying '{topic}'")
            if topic.endswith("tf_static"):
                self.subs[topic] = self.create_subscription(
                    TFMessage, topic, self.static_callback, STATIC_QOS
                )
            else:
                self.msg_counts[robot] = 0
                self.subs[topic] = self.create_subscription(
                    TFMessage,
                    topic,
                    lambda msg, robot=robot: self.tf_callback(msg, robot),
                    DYNAMIC_QOS,
                )

    def tf_callback(self, msg, robot):
        self.last_msg_time[robot] = time.monotonic()
        self.msg_counts[robot] += 1
        self.tf_pub.publish(msg)

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

        robots = sorted(self.msg_counts)
        stale = []
        rates = []
        for robot in robots:
            last = self.last_msg_time.get(robot)
            if last is None or t_now - last > self.stale_after_s:
                age = "never" if last is None else f"{t_now - last:.0f} s ago"
                stale.append(f"{robot} ({age})")
            else:
                rates.append(f"{robot} {self.msg_counts[robot] / dt:.1f} Hz")

            self.msg_counts[robot] = 0

        if not robots:
            status = NodeInfoMsg.WARNING
            notes = "No robots found on /<robot>/network/tf"
        elif stale:
            status = NodeInfoMsg.WARNING
            notes = f"Stale TF: {', '.join(stale)}"
            if rates:
                notes += f". OK: {', '.join(rates)}"
        else:
            status = NodeInfoMsg.NOMINAL
            notes = f"Relaying {', '.join(rates)}"

        msg = NodeInfoMsg()
        msg.nickname = "tf_relay"
        msg.node_name = self.get_fully_qualified_name()
        msg.status = status
        msg.notes = notes
        self.heartbeat_pub.publish(msg)


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
