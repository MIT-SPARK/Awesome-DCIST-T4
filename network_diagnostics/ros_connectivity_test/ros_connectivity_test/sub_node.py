#!/usr/bin/env python3

import json
import os
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from std_msgs.msg import String


class SourceTracker:
    def __init__(self):
        self.count = 0
        self.last_seq = -1
        self.dropped = 0
        self.last_seen = 0.0
        self.latencies = []

    def update(self, seq, send_time):
        now = time.time()
        self.count += 1
        self.latencies.append(now - send_time)
        if len(self.latencies) > 100:
            self.latencies.pop(0)
        if self.last_seq >= 0 and seq > self.last_seq + 1:
            self.dropped += seq - self.last_seq - 1
        self.last_seq = seq
        self.last_seen = now

    @property
    def avg_latency(self):
        if not self.latencies:
            return 0.0
        return sum(self.latencies) / len(self.latencies)

    @property
    def loss_rate(self):
        total = self.count + self.dropped
        if total == 0:
            return 0.0
        return self.dropped / total

    def percentiles(self):
        if not self.latencies:
            return {'p50': 0.0, 'p95': 0.0, 'p99': 0.0, 'max': 0.0, 'min': 0.0}
        s = sorted(self.latencies)
        n = len(s)
        return {
            'p50': s[n // 2],
            'p95': s[min(int(n * 0.95), n - 1)],
            'p99': s[min(int(n * 0.99), n - 1)],
            'max': s[-1],
            'min': s[0],
        }


class ConnectivitySubNode(Node):

    def __init__(self):
        super().__init__('connectivity_sub')
        self.declare_parameters(
            namespace='',
            parameters=[
                ('silent_threshold_sec', 5.0),
                ('report_rate_hz', 0.5),
            ]
        )

        self.silent_threshold = self.get_parameter(
            'silent_threshold_sec'
        ).get_parameter_value().double_value
        report_rate = self.get_parameter(
            'report_rate_hz'
        ).get_parameter_value().double_value

        self.robot_name = os.environ.get('ADT4_ROBOT_NAME', os.uname().nodename)
        self.sources = {}

        self.status_pub = self.create_publisher(
            String,
            '/connectivity_test/status',
            qos_profile=QoSProfile(depth=10),
        )

        # Subscribe to all heartbeats using a wildcard-compatible topic pattern.
        # ROS2 doesn't support wildcard subscriptions natively, so we discover
        # topics dynamically via a periodic check.
        self.known_topics = set()
        self.create_timer(2.0, self.discover_topics)
        self.create_timer(1.0 / report_rate, self.report_status)

        self.get_logger().info('Connectivity subscriber started, discovering heartbeat topics...')

    def discover_topics(self):
        topic_list = self.get_topic_names_and_types()
        for topic_name, _ in topic_list:
            if topic_name.endswith('/connectivity_test/heartbeat') and topic_name not in self.known_topics:
                self.known_topics.add(topic_name)
                self.create_subscription(
                    String,
                    topic_name,
                    self.heartbeat_cb,
                    QoSProfile(depth=10),
                )
                self.get_logger().info(f'Subscribed to {topic_name}')

    def heartbeat_cb(self, msg):
        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError:
            return

        src = data.get('src', 'unknown')
        seq = data.get('seq', 0)
        ts = data.get('timestamp', time.time())

        if src not in self.sources:
            self.sources[src] = SourceTracker()
            self.get_logger().info(f'New source discovered: {src}')

        self.sources[src].update(seq, ts)

    def report_status(self):
        now = time.time()
        report = {}

        for src, tracker in self.sources.items():
            silent_for = now - tracker.last_seen if tracker.last_seen > 0 else float('inf')
            alive = silent_for < self.silent_threshold

            if not alive:
                self.get_logger().warn(f'Source {src} silent for {silent_for:.1f}s')

            pct = tracker.percentiles()
            report[src] = {
                'alive': alive,
                'count': tracker.count,
                'dropped': tracker.dropped,
                'loss_rate': round(tracker.loss_rate, 4),
                'avg_latency_ms': round(tracker.avg_latency * 1000, 2),
                'p50_latency_ms': round(pct['p50'] * 1000, 2),
                'p95_latency_ms': round(pct['p95'] * 1000, 2),
                'p99_latency_ms': round(pct['p99'] * 1000, 2),
                'max_latency_ms': round(pct['max'] * 1000, 2),
                'min_latency_ms': round(pct['min'] * 1000, 2),
                'silent_for_sec': round(silent_for, 2),
            }

        if report:
            status_msg = String()
            status_msg.data = json.dumps({
                'reporter': self.robot_name,
                'timestamp': now,
                'sources': report,
            })
            self.status_pub.publish(status_msg)

            # Log summary
            lines = [f'  {src}: {"OK" if d["alive"] else "SILENT"} '
                     f'(rx={d["count"]}, drop={d["dropped"]}, '
                     f'lat={d["avg_latency_ms"]:.1f}ms, '
                     f'p95={d["p95_latency_ms"]:.1f}ms)'
                     for src, d in report.items()]
            self.get_logger().info('Connectivity status:\n' + '\n'.join(lines))


def main(args=None):
    rclpy.init(args=args)
    node = ConnectivitySubNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
