#!/usr/bin/env python3
"""Forward AWCD (Active Window Change Detector) reports to the scene change writer.

Input   ~/awcd_changes       khronos_msgs/AwcdChanges: the AWCD base station's reconciled
                             changes (in the map frame, merged across robots, with class
                             names), or every robot's raw /awcd_changes
        /<robot>/labelspace  semantic_inference_msgs/Labelspace (latched), for class names
                             of raw reports
Output  ~/scene_changes      heracles_ros_interfaces/SceneChangeMsg, source "awcd/<robot>"

The writer applies the changes to Neo4j and announces them on /scene_graph/changes_applied,
where the repair planner judges whether they affect the running plan. Which changes are sent,
and when, is decided in awcd_scene_changes.py. By default (send_full_change_set) each robot's
whole current set is sent whenever it changes, empty sets included, so the writer can undo
what AWCD withdrew -- a change detection reset restores the prior map. With it false, each
change is sent once and never undone (for a writer without the change log).

Raw added objects are held until the robot's labelspace arrives, so they enter the map with
their class; after labelspace_timeout_s without one they are sent unlabelled.

Raw reports are in each robot's frame. Only the base station transforms them into the map, so
on hardware the input should be its reconciled changes.
"""

import time
from typing import Dict

import rclpy
from geometry_msgs.msg import Point
from heracles_ros_interfaces.msg import SceneChange, SceneChangeMsg
from khronos_msgs.msg import AwcdChanges
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from semantic_inference_msgs.msg import Labelspace

from dcist_launch_system.awcd_change_store import RECONCILED
from dcist_launch_system.awcd_scene_changes import (
    AwcdSceneChanges,
    ReportedObject,
    labelspace_lookup,
    snapshot_signature,
)


def _reported(info, added: bool) -> ReportedObject:
    obj = ReportedObject(
        obj_id=int(info.id),
        change_confidence=float(info.change_confidence),
        num_frames_observed=int(info.num_frames_observed),
    )
    if added:
        d = info.bbox_dimensions
        c = info.bbox_center if (d.x > 0 or d.y > 0 or d.z > 0) else info.centroid
        obj.position = (float(c.x), float(c.y), float(c.z))
        obj.semantic_label = int(info.semantic_label)
        obj.name = str(info.name)
    return obj


class AwcdSceneChangeBridge(Node):
    def __init__(self):
        super().__init__("awcd_scene_change_bridge")
        self.declare_parameter("min_change_confidence", 0.0)
        self.declare_parameter("min_frames", 0)
        self.declare_parameter("labelspace_topic", "/{robot}/labelspace")
        self.declare_parameter("labelspace_timeout_s", 10.0)
        self.declare_parameter("send_full_change_set", True)
        self._full = bool(self.get_parameter("send_full_change_set").value)
        self._last_sent: Dict[
            str, tuple
        ] = {}  # robot -> signature of the set last sent
        self.filter = AwcdSceneChanges(
            float(self.get_parameter("min_change_confidence").value),
            int(self.get_parameter("min_frames").value),
        )
        self._labelspace_topic = str(self.get_parameter("labelspace_topic").value)
        self._labelspace_timeout_s = float(
            self.get_parameter("labelspace_timeout_s").value
        )

        self._labels: Dict[str, object] = {}  # robot -> category id to name
        self._first_report: Dict[str, float] = {}
        # Latest report per robot: AWCD republishes only when its id set changes, so held-back
        # added objects are re-offered from here when the labelspace (or the timeout) arrives.
        self._latest: Dict[str, AwcdChanges] = {}

        self._pub = self.create_publisher(SceneChangeMsg, "~/scene_changes", 10)
        # Volatile, though the base station's output is latched: what was sent is not kept
        # across a restart, so a report received again would add its objects twice.
        self._sub = self.create_subscription(
            AwcdChanges, "~/awcd_changes", self._on_changes, 10
        )
        self.create_timer(1.0, self._retry_held_back)
        self.get_logger().info(f"Forwarding {self._sub.topic_name} to ~/scene_changes")

    def _on_changes(self, msg: AwcdChanges):
        robot = msg.robot_name
        if robot not in self._first_report:
            self._first_report[robot] = time.monotonic()
            if robot != RECONCILED:  # reconciled reports carry class names
                self._subscribe_labelspace(robot)
        self._latest[robot] = msg
        self._ingest(msg)

    def _subscribe_labelspace(self, robot: str):
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        topic = self._labelspace_topic.format(robot=robot)
        self.create_subscription(
            Labelspace, topic, lambda m, r=robot: self._on_labelspace(r, m), qos
        )

    def _on_labelspace(self, robot: str, msg: Labelspace):
        self._labels[robot] = labelspace_lookup(list(msg.names))
        self.get_logger().info(f"{robot}: labelspace with {len(msg.names)} names")
        if robot in self._latest:
            self._ingest(self._latest[robot])

    def _label_name(self, robot: str):
        if robot in self._labels:
            return self._labels[robot]
        if time.monotonic() - self._first_report[robot] >= self._labelspace_timeout_s:
            return lambda _label: ""
        return None

    def _retry_held_back(self):
        for robot, msg in self._latest.items():
            added = [_reported(i, True) for i in msg.added_objects]
            if self.filter.held_back(robot, added):
                self._ingest(msg)

    def _ingest(self, msg: AwcdChanges):
        robot = msg.robot_name
        removed = [_reported(i, False) for i in msg.removed_objects]
        added = [_reported(i, True) for i in msg.added_objects]
        label_name = self._label_name(robot)
        if self._full:
            changes = self.filter.snapshot(robot, removed, added, label_name)
            signature = snapshot_signature(changes)
            if self._last_sent.get(robot) == signature:
                return
            self._last_sent[robot] = signature
        else:
            changes = self.filter.changes(robot, removed, added, label_name)
        if label_name is None and self.filter.held_back(robot, added):
            self.get_logger().warn(
                f"{robot}: holding added objects until its labelspace arrives",
                throttle_duration_sec=5.0,
            )
        if not changes and not self._full:
            return
        if msg.header.frame_id and msg.header.frame_id != "map":
            self.get_logger().warn(
                f"{robot} reports in '{msg.header.frame_id}', the map is in 'map'",
                throttle_duration_sec=30.0,
            )
        out = SceneChangeMsg()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = "map"
        out.source = f"awcd/{robot}"
        for c in changes:
            sc = SceneChange()
            sc.kind = c.kind
            sc.symbol = c.symbol
            sc.semantic_label = c.semantic_label
            x, y, z = c.position
            sc.new_position = Point(x=x, y=y, z=z)
            sc.confidence = c.confidence
            out.changes.append(sc)
        self._pub.publish(out)
        if not changes:
            self.get_logger().info(f"{robot}: sent an empty change set")
            return
        self.get_logger().info(
            f"{robot}: sent "
            + ("full set: " if self._full else "")
            + ", ".join(
                f"removed {c.symbol}"
                if c.kind == SceneChange.REMOVED
                else f"added {c.semantic_label or 'unlabelled'}"
                for c in changes
            )
        )


def main(args=None):
    rclpy.init(args=args)
    node = AwcdSceneChangeBridge()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()  # a SIGTERM has already shut the context down


if __name__ == "__main__":
    main()
