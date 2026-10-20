#!/usr/bin/env python3
"""Republish this robot's TF at a reduced rate for the base station.

Subscribes to tf / tf_static in the node's namespace (/<robot>/tf) and publishes
network/tf / network/tf_static (/<robot>/network/tf), which is the only TF the base
station's tf_relay subscribes to. The full-rate TF therefore never crosses the network.

Two modes:
- frames (default): only forward reference_frame -> <robot>/<frame> for each of
  `frames` (e.g. map -> hamilton/base_link), which is what the base station looks up
- all: forward the whole tree, throttled per frame
"""

import time

import rclpy
import tf2_ros
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from ros_system_monitor_msgs.msg import NodeInfoMsg
from tf2_msgs.msg import TFMessage

from dcist_launch_system.tf_relay_node import DYNAMIC_QOS, STATIC_QOS, STATIC_SUB_QOS

MODES = ("frames", "all")


class TfNetwork(Node):
    def __init__(self):
        super().__init__("tf_network")

        self.declare_parameter("mode", "frames")
        self.declare_parameter("rate_hz", 10.0)
        self.declare_parameter("reference_frame", "map")
        self.declare_parameter("frames", ["base_link", "map"])
        self.declare_parameter("stale_after_s", 2.0)

        self.mode = self.get_parameter("mode").get_parameter_value().string_value
        if self.mode not in MODES:
            self.get_logger().error(f"Unknown mode '{self.mode}', using 'frames'")
            self.mode = "frames"

        rate_hz = self.get_parameter("rate_hz").get_parameter_value().double_value
        self.stale_after_s = (
            self.get_parameter("stale_after_s").get_parameter_value().double_value
        )
        self.reference_frame = (
            self.get_parameter("reference_frame").get_parameter_value().string_value
        )
        # frames without a robot prefix are relative to this robot
        robot = self.get_namespace().strip("/")
        frames = self.get_parameter("frames").get_parameter_value().string_array_value
        self.frames_to_send = [
            f if "/" in f or not robot else f"{robot}/{f}" for f in frames
        ]

        # all mode: /tf mixes publishers at different rates, so we throttle per frame
        # (keeping the latest transform for each child frame) instead of dropping
        # whole messages
        self.pending = {}
        self.static_transforms = {}

        # frames mode: look up each frame from the robot's own TF
        self.buffer = tf2_ros.Buffer(cache_time=Duration(seconds=10.0))
        self.last_stamp = {}
        self.last_update = {}
        self.lookup_errors = {}

        self.tf_pub = self.create_publisher(TFMessage, "network/tf", DYNAMIC_QOS)
        self.static_pub = self.create_publisher(
            TFMessage, "network/tf_static", STATIC_QOS
        )
        self.create_subscription(TFMessage, "tf", self.tf_callback, DYNAMIC_QOS)
        self.create_subscription(
            TFMessage, "tf_static", self.static_callback, STATIC_SUB_QOS
        )
        flush = self.flush_all if self.mode == "all" else self.flush_frames
        self.timer = self.create_timer(1.0 / rate_hz, flush)

        # TF stats for the status monitor. Uses the steady clock so sim time and paused
        # bags don't affect staleness
        self.frames = set()
        self.last_msg_time = None
        self.n_in = 0
        self.n_out = 0
        self.last_hb_time = time.monotonic()

        self.heartbeat_pub = self.create_publisher(NodeInfoMsg, "~/node_status", 1)
        self.hb_timer = self.create_timer(1.0, self.hb_callback)

        if self.mode == "all":
            self.get_logger().info("Forwarding the whole TF tree")
        else:
            targets = ", ".join(self.frames_to_send)
            self.get_logger().info(f"Forwarding {self.reference_frame} -> [{targets}]")

    def tf_callback(self, msg):
        self.last_msg_time = time.monotonic()
        self.n_in += 1
        for tf in msg.transforms:
            self.frames.add(tf.child_frame_id)
            if self.mode == "all":
                self.pending[tf.child_frame_id] = tf
            else:
                self.buffer.set_transform(tf, "tf_network")

    def static_callback(self, msg):
        for tf in msg.transforms:
            if self.mode == "all":
                self.static_transforms[tf.child_frame_id] = tf
            else:
                self.buffer.set_transform_static(tf, "tf_network")

        if self.mode == "all":
            self._publish_static()

    def _publish_static(self):
        self.static_pub.publish(
            TFMessage(transforms=list(self.static_transforms.values()))
        )

    def flush_all(self):
        # only send frames that updated, as resending old stamps makes tf2 warn
        if not self.pending:
            return

        self.tf_pub.publish(TFMessage(transforms=list(self.pending.values())))
        self.pending = {}
        self.n_out += 1

    def flush_frames(self):
        dynamic = []
        static_changed = False
        for frame in self.frames_to_send:
            try:
                tf = self.buffer.lookup_transform(self.reference_frame, frame, Time())
            except tf2_ros.TransformException as e:
                self.lookup_errors[frame] = str(e).split(".")[0]
                continue

            self.lookup_errors.pop(frame, None)
            stamp = Time.from_msg(tf.header.stamp).nanoseconds
            if stamp == 0:
                # the whole chain is static: send it latched, and only when it changes
                old = self.static_transforms.get(frame)
                if old is None or old.transform != tf.transform:
                    self.static_transforms[frame] = tf
                    static_changed = True
                self.last_update[frame] = time.monotonic()
                continue

            # only send frames that updated, as resending old stamps makes tf2 warn
            if stamp > self.last_stamp.get(frame, -1):
                self.last_stamp[frame] = stamp
                self.last_update[frame] = time.monotonic()
                dynamic.append(tf)

        if dynamic:
            self.tf_pub.publish(TFMessage(transforms=dynamic))
            self.n_out += 1

        if static_changed:
            self._publish_static()

    def _frames_status(self, t_now):
        """Warnings and per-frame notes for frames mode."""
        problems = []
        ok = []
        for frame in self.frames_to_send:
            name = f"{self.reference_frame}->{frame}"
            if frame in self.lookup_errors:
                problems.append(f"no {name}")
            elif frame in self.static_transforms:
                ok.append(f"{name} static")
            elif t_now - self.last_update.get(frame, 0.0) > self.stale_after_s:
                problems.append(f"{name} stale")
            else:
                ok.append(name)

        return problems, ok

    def hb_callback(self):
        t_now = time.monotonic()
        dt = t_now - self.last_hb_time
        self.last_hb_time = t_now
        rate_in = self.n_in / dt
        rate_out = self.n_out / dt
        self.n_in = 0
        self.n_out = 0

        # a subscriber means the base station relay is connected
        n_subs = self.tf_pub.get_subscription_count()
        connected = "base station connected" if n_subs else "no subscribers"

        input_topic = self.resolve_topic_name("tf")
        if self.last_msg_time is None:
            status = NodeInfoMsg.WARNING
            notes = f"No TF received on {input_topic}"
        elif t_now - self.last_msg_time > self.stale_after_s:
            status = NodeInfoMsg.WARNING
            age = t_now - self.last_msg_time
            notes = f"No TF on {input_topic} for {age:.0f} s"
        elif self.mode == "all":
            status = NodeInfoMsg.NOMINAL
            notes = (
                f"all: {len(self.frames)} frames, {rate_in:.0f} Hz in, "
                f"{rate_out:.1f} Hz out ({connected})"
            )
        else:
            problems, ok = self._frames_status(t_now)
            status = NodeInfoMsg.WARNING if problems else NodeInfoMsg.NOMINAL
            notes = ", ".join(problems + ok)
            notes += f", {rate_out:.1f} Hz out ({connected})"

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
