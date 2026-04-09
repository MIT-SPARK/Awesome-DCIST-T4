#!/usr/bin/env python3

import json
import os
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from std_msgs.msg import String


class ConnectivityPubNode(Node):
    def __init__(self):
        super().__init__("connectivity_pub")
        self.declare_parameters(
            namespace="",
            parameters=[
                ("publish_rate_hz", 1.0),
            ],
        )

        publish_rate_hz = (
            self.get_parameter("publish_rate_hz").get_parameter_value().double_value
        )
        self.robot_name = os.environ.get("ADT4_ROBOT_NAME", os.uname().nodename)
        self.seq = 0

        self.pub = self.create_publisher(
            String,
            f"/{self.robot_name}/connectivity_test/heartbeat",
            qos_profile=QoSProfile(depth=10),
        )

        self.timer = self.create_timer(1.0 / publish_rate_hz, self.timer_cb)
        self.get_logger().info(
            f"Publishing heartbeats on /{self.robot_name}/connectivity_test/heartbeat "
            f"at {publish_rate_hz} Hz"
        )

    def timer_cb(self):
        msg = String()
        msg.data = json.dumps(
            {
                "src": self.robot_name,
                "seq": self.seq,
                "timestamp": time.time(),
            }
        )
        self.pub.publish(msg)
        self.seq += 1


def main(args=None):
    rclpy.init(args=args)
    node = ConnectivityPubNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
