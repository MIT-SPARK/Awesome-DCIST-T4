import importlib

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node


class DataPublisher:
    def __init__(self, node, input_topic, msg_type_str, publish_rate):
        self.node = node
        self.input_topic = input_topic
        self.output_topic = f"/downsampled{input_topic}"
        self.msg_type = self._resolve_msg_type(msg_type_str)
        self.publish_rate = publish_rate

        self.last_msg = None

        self.publisher = self.node.create_publisher(
            self.msg_type, self.output_topic, 10
        )
        self.subscriber = self.node.create_subscription(
            self.msg_type, self.input_topic, self.callback, 10
        )
        self.timer = self.node.create_timer(
            1.0 / self.publish_rate, self.publish_callback
        )

    def callback(self, msg):
        self.last_msg = msg

    def publish_callback(self):
        if self.last_msg:
            self.publisher.publish(self.last_msg)
            self.node.get_logger().info(
                f"Relaying {self.input_topic} to {self.output_topic} @ {self.publish_rate}Hz."
            )

    def _resolve_msg_type(self, msg_type_str):
        pkg, _, msg = msg_type_str.split("/")
        module = importlib.import_module(f"{pkg}.msg")
        return getattr(module, msg)


class DataRelayNode(Node):
    def __init__(self):
        super().__init__("data_relay")

        relay_configs = [("/tf", "tf2_msgs/msg/TFMessage", 2.0)]
        for config in relay_configs:
            topic, msg_type_str, rate = config

            DataPublisher(self, topic, msg_type_str, rate)


def main(args=None):
    rclpy.init(args=args)
    try:
        node = DataRelayNode()

        executor = MultiThreadedExecutor()
        executor.add_node(node)

        try:
            executor.spin()
        finally:
            executor.shutdown()
            node.destroy_node()
    finally:
        rclpy.shutdown()
