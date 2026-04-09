#!/usr/bin/env python3
"""ROS2 topic bandwidth monitor — tracks bytes/sec across discovered topics."""

import json
import os
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from std_msgs.msg import String


class TopicBwTracker:
    def __init__(self):
        self.bytes_total = 0
        self.msg_count = 0
        self.window_bytes = 0
        self.window_msgs = 0
        self.window_start = time.time()

    def record(self, size):
        self.bytes_total += size
        self.msg_count += 1
        self.window_bytes += size
        self.window_msgs += 1

    def reset_window(self):
        now = time.time()
        elapsed = now - self.window_start
        bps = self.window_bytes / elapsed if elapsed > 0 else 0
        mps = self.window_msgs / elapsed if elapsed > 0 else 0
        self.window_bytes = 0
        self.window_msgs = 0
        self.window_start = now
        return bps, mps, elapsed


class BwMonitorNode(Node):
    def __init__(self):
        super().__init__("bw_monitor")
        self.declare_parameters(
            namespace="",
            parameters=[
                ("report_rate_hz", 1.0),
                ("topic_filter", ""),
                ("exclude_filter", "/rosout,/parameter_events"),
            ],
        )

        report_rate = (
            self.get_parameter("report_rate_hz").get_parameter_value().double_value
        )
        self.topic_filter = (
            self.get_parameter("topic_filter").get_parameter_value().string_value
        )
        self.exclude_filter = set(
            self.get_parameter("exclude_filter")
            .get_parameter_value()
            .string_value.split(",")
        )

        self.robot_name = os.environ.get("ADT4_ROBOT_NAME", os.uname().nodename)
        self.trackers = {}  # topic -> TopicBwTracker
        self.subscriptions_map = {}  # topic -> subscription

        self.bw_pub = self.create_publisher(
            String,
            "/connectivity_test/bandwidth",
            qos_profile=QoSProfile(depth=10),
        )

        self.create_timer(5.0, self.discover_topics)
        self.create_timer(1.0 / report_rate, self.report_bandwidth)

        self.get_logger().info("Bandwidth monitor started")

    def discover_topics(self):
        topic_list = self.get_topic_names_and_types()
        for topic_name, topic_types in topic_list:
            if topic_name in self.subscriptions_map:
                continue

            # Apply filters
            if topic_name in self.exclude_filter:
                continue
            if self.topic_filter and self.topic_filter not in topic_name:
                continue

            # Only subscribe to std_msgs/String topics for generic monitoring
            # For other types, we can still measure using serialized size
            if "std_msgs/msg/String" in topic_types:
                self._subscribe_string(topic_name)
            # Skip other types to avoid import issues — the connectivity
            # heartbeat and status topics use String, which is the primary
            # use case

    def _subscribe_string(self, topic_name):
        qos = QoSProfile(
            depth=10,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
        )
        self.trackers[topic_name] = TopicBwTracker()

        def cb(msg, tn=topic_name):
            self.trackers[tn].record(len(msg.data.encode()))

        sub = self.create_subscription(String, topic_name, cb, qos)
        self.subscriptions_map[topic_name] = sub
        self.get_logger().info(f"Monitoring bandwidth: {topic_name}")

    def report_bandwidth(self):
        if not self.trackers:
            return

        report = {}
        total_bps = 0.0
        total_mps = 0.0

        for topic, tracker in self.trackers.items():
            bps, mps, elapsed = tracker.reset_window()
            total_bps += bps
            total_mps += mps
            report[topic] = {
                "bytes_per_sec": round(bps, 1),
                "msgs_per_sec": round(mps, 2),
                "total_bytes": tracker.bytes_total,
                "total_msgs": tracker.msg_count,
            }

        report["_summary"] = {
            "total_bytes_per_sec": round(total_bps, 1),
            "total_msgs_per_sec": round(total_mps, 2),
            "topics_monitored": len(self.trackers),
        }

        bw_msg = String()
        bw_msg.data = json.dumps(
            {
                "reporter": self.robot_name,
                "timestamp": time.time(),
                "bandwidth": report,
            }
        )
        self.bw_pub.publish(bw_msg)

        # Log summary
        def fmt_bytes(b):
            if b >= 1024 * 1024:
                return f"{b / (1024 * 1024):.1f} MB/s"
            if b >= 1024:
                return f"{b / 1024:.1f} KB/s"
            return f"{b:.0f} B/s"

        lines = []
        for topic, data in sorted(report.items()):
            if topic == "_summary":
                continue
            lines.append(
                f"  {topic}: {fmt_bytes(data['bytes_per_sec'])} "
                f"({data['msgs_per_sec']:.1f} msg/s)"
            )
        lines.append(
            f"  TOTAL: {fmt_bytes(total_bps)} "
            f"({total_mps:.1f} msg/s, "
            f"{len(self.trackers)} topics)"
        )
        self.get_logger().info("Bandwidth:\n" + "\n".join(lines))


def main(args=None):
    rclpy.init(args=args)
    node = BwMonitorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
