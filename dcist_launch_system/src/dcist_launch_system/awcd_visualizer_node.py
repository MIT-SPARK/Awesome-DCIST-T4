#!/usr/bin/env python3
"""Base-station visualizer for Active Window Change Detector (AWCD) reports.

Subscribes to the base station's own prior DSG (published by prior_dsg_publisher_node) and
to the global `khronos_msgs/AwcdChanges` topic that every robot's
`ActiveWindowChangeDetectorPublisher` publishes to, and draws red (removed) / green (added)
bounding-box wireframes + text labels into RViz -- the base-station equivalent of
`ActiveWindowChangeDetectorVisualizer` (khronos_ros), which only runs on-robot.

Reports from all robots are kept as a latest-message-per-robot snapshot in an `AwcdChangeStore`
(see awcd_change_store.py), keyed by (robot_name, kind) rather than rendered directly inline, so
the node retains per-robot provenance for future cross-robot filtering/merging -- but each robot's
message fully replaces that robot's previous one, since the robot already publishes its complete,
refined change set every time.
"""

import time
from typing import Dict, List, Tuple

import numpy as np
import rclpy
import tf2_ros
import yaml
from geometry_msgs.msg import Point
from hydra_ros import DsgSubscriber
from khronos_msgs.msg import AwcdChanges
from rclpy.node import Node
from rclpy.time import Time
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray

from dcist_launch_system.awcd_change_store import (
    ADDED,
    REMOVED,
    AwcdChangeStore,
    ChangeRecord,
)

RED = ColorRGBA(r=1.0, g=0.0, b=0.0, a=1.0)
GREEN = ColorRGBA(r=0.0, g=1.0, b=0.0, a=1.0)


def _load_labelspace(filepath: str) -> Dict[int, str]:
    if not filepath:
        return {}
    try:
        with open(filepath, "r") as f:
            data = yaml.safe_load(f)
        return {
            int(entry["label"]): entry["name"] for entry in data.get("label_names", [])
        }
    except (OSError, yaml.YAMLError, KeyError, TypeError) as exc:
        raise RuntimeError(
            f"Failed to load labelspace from '{filepath}': {exc}"
        ) from exc


def _bbox_line_list_points(dimensions: np.ndarray) -> List[Point]:
    """12-edge / 24-point axis-aligned wireframe, mirroring khronos_ros setBoundingBox()."""
    base = -dimensions / 2.0
    dx = np.array([dimensions[0], 0.0, 0.0])
    dy = np.array([0.0, dimensions[1], 0.0])
    dz = np.array([0.0, 0.0, dimensions[2]])

    def p(offset: np.ndarray) -> Point:
        v = base + offset
        return Point(x=float(v[0]), y=float(v[1]), z=float(v[2]))

    zero = np.zeros(3)
    return [
        p(zero),
        p(dx),
        p(zero),
        p(dy),
        p(dx),
        p(dx + dy),
        p(dy),
        p(dx + dy),
        p(dz),
        p(dx + dz),
        p(dz),
        p(dy + dz),
        p(dx + dz),
        p(dx + dy + dz),
        p(dy + dz),
        p(dx + dy + dz),
        p(zero),
        p(dz),
        p(dx),
        p(dx + dz),
        p(dy),
        p(dy + dz),
        p(dx + dy),
        p(dx + dy + dz),
    ]


class AwcdVisualizerNode(Node):
    def __init__(self):
        super().__init__("awcd_visualizer")

        self.declare_parameter("target_frame", "map")
        self.declare_parameter("changes_topic", "/awcd_changes")
        self.declare_parameter("labelspace_filepath", "")
        self.declare_parameter("line_width", 0.1)
        self.declare_parameter("text_scale", 0.4)
        self.declare_parameter("show_labels", True)
        self.declare_parameter("tf_fallback_identity", True)

        self.target_frame = (
            self.get_parameter("target_frame").get_parameter_value().string_value
        )
        changes_topic = (
            self.get_parameter("changes_topic").get_parameter_value().string_value
        )
        self.line_width = (
            self.get_parameter("line_width").get_parameter_value().double_value
        )
        self.text_scale = (
            self.get_parameter("text_scale").get_parameter_value().double_value
        )
        self.show_labels = (
            self.get_parameter("show_labels").get_parameter_value().bool_value
        )
        self.tf_fallback_identity = (
            self.get_parameter("tf_fallback_identity").get_parameter_value().bool_value
        )
        labelspace_filepath = (
            self.get_parameter("labelspace_filepath").get_parameter_value().string_value
        )
        self.labelspace = _load_labelspace(labelspace_filepath)

        self.graph = None
        self.store = AwcdChangeStore()
        # Latest raw message per robot, so we can re-ingest (rebuild removed records) once the
        # prior DSG arrives, without waiting for the robot's next (change-gated) publish.
        self._latest_msgs: Dict[str, AwcdChanges] = {}
        # Marker ids emitted for each (robot_name, marker_ns) in the previous publish, so we can
        # emit DELETEs for ids that dropped out -- python equivalent of MarkerTracker::clearPrevious.
        self._prev_marker_ids: Dict[Tuple[str, str], set] = {}

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.dsg_sub = DsgSubscriber(self, "~/dsg_in", self._on_dsg)
        self.changes_sub = self.create_subscription(
            AwcdChanges, changes_topic, self._on_changes, 10
        )
        self.marker_pub = self.create_publisher(MarkerArray, "~/changed_objects", 10)

    def _on_dsg(self, header, graph):
        self.graph = graph
        # Removed-object records need the prior DSG to resolve bboxes; re-ingest every cached
        # message now so removed objects appear immediately instead of waiting for the next
        # change-gated publish from each robot.
        for msg in self._latest_msgs.values():
            self._ingest(msg)

    def _lookup_prior_T_msg(self, msg_frame_id: str):
        """Return 4x4 transform target_frame_T_msg_frame, or identity + warn on TF failure."""
        if msg_frame_id == self.target_frame:
            return np.eye(4)
        try:
            tf = self.tf_buffer.lookup_transform(
                self.target_frame, msg_frame_id, Time()
            )
        except tf2_ros.TransformException as exc:
            if not self.tf_fallback_identity:
                return None
            self.get_logger().warn(
                f"TF lookup {self.target_frame} <- {msg_frame_id} failed ({exc}); "
                "falling back to identity (frames assumed coincident).",
                throttle_duration_sec=5.0,
            )
            return np.eye(4)

        t = tf.transform.translation
        q = tf.transform.rotation
        return _quat_trans_to_matrix(q.x, q.y, q.z, q.w, t.x, t.y, t.z)

    def _on_changes(self, msg: AwcdChanges):
        self._latest_msgs[msg.robot_name] = msg
        self._ingest(msg)

    def _ingest(self, msg: AwcdChanges):
        transform = self._lookup_prior_T_msg(msg.header.frame_id)
        if transform is None:
            return

        removed_records = self._build_removed_records(msg, transform)
        # Added-object attributes are self-contained in the message (no prior-DSG lookup needed),
        # so this never depends on graph availability.
        added_records = self._build_added_records(msg, transform)

        # The message is authoritative for this robot -- always replace both slices, including
        # with an empty removed list when the prior DSG isn't available yet, so no stale removed
        # objects from an earlier message linger indefinitely.
        self.store.update(msg.robot_name, REMOVED, removed_records)
        self.store.update(msg.robot_name, ADDED, added_records)

        self._publish_markers()

    def _build_removed_records(self, msg: AwcdChanges, transform: np.ndarray):
        if self.graph is None:
            self.get_logger().warn(
                "No prior DSG received yet; skipping removed objects from this message "
                f"(robot={msg.robot_name}). These will be rebuilt once the prior DSG arrives.",
                throttle_duration_sec=5.0,
            )
            return []

        now_ns = time.time_ns()
        records = []
        for info in msg.removed_objects:
            node = self.graph.find_node(int(info.id))
            if node is None:
                self.get_logger().warn(
                    f"Removed object {info.id} from {msg.robot_name} not found in prior DSG; "
                    "skipping.",
                    throttle_duration_sec=5.0,
                )
                continue
            attrs = node.attributes
            bbox = getattr(attrs, "bounding_box", None)
            if bbox is None or not bbox.is_valid():
                self.get_logger().warn(
                    f"Removed object {info.id} from {msg.robot_name} has no valid bounding box "
                    "in the prior DSG; skipping.",
                    throttle_duration_sec=5.0,
                )
                continue

            center = _transform_point(
                transform, np.array(bbox.world_P_center, dtype=float)
            )
            dims = np.array(bbox.dimensions, dtype=float)
            semantic_label = getattr(attrs, "semantic_label", -1)
            records.append(
                ChangeRecord(
                    robot_name=msg.robot_name,
                    kind=REMOVED,
                    obj_id=int(info.id),
                    stamp_ns=Time.from_msg(info.stamp).nanoseconds,
                    center=center,
                    dimensions=dims,
                    orientation=np.array([0.0, 0.0, 0.0, 1.0]),
                    semantic_label=int(semantic_label),
                    confidence=float(info.confidence),
                    change_confidence=float(info.change_confidence),
                    num_frames_observed=int(info.num_frames_observed),
                    last_seen_ns=now_ns,
                )
            )
        return records

    def _build_added_records(self, msg: AwcdChanges, transform: np.ndarray):
        now_ns = time.time_ns()
        records = []
        for info in msg.added_objects:
            center = _transform_point(
                transform,
                np.array([info.bbox_center.x, info.bbox_center.y, info.bbox_center.z]),
            )
            dims = np.array(
                [info.bbox_dimensions.x, info.bbox_dimensions.y, info.bbox_dimensions.z]
            )
            orientation = np.array(
                [
                    info.bbox_orientation.x,
                    info.bbox_orientation.y,
                    info.bbox_orientation.z,
                    info.bbox_orientation.w,
                ]
            )
            records.append(
                ChangeRecord(
                    robot_name=msg.robot_name,
                    kind=ADDED,
                    obj_id=int(info.id),
                    stamp_ns=Time.from_msg(info.stamp).nanoseconds,
                    center=center,
                    dimensions=dims,
                    orientation=orientation,
                    semantic_label=int(info.semantic_label),
                    confidence=float(info.confidence),
                    change_confidence=float(info.change_confidence),
                    num_frames_observed=int(info.num_frames_observed),
                    last_seen_ns=now_ns,
                )
            )
        return records

    def _label_text(self, record: ChangeRecord) -> str:
        if record.semantic_label < 0:
            label_name = "unknown"
        else:
            label_name = self.labelspace.get(
                record.semantic_label, f"label_{record.semantic_label}"
            )
        return (
            f"{record.robot_name} | {record.kind} {record.obj_id} | {label_name} | "
            f"cc={record.change_confidence:.2f} n={record.num_frames_observed}"
        )

    def _publish_markers(self):
        array = MarkerArray()
        cur_ids: Dict[Tuple[str, str], set] = {}
        zero_stamp = Time().to_msg()

        for robot_name in self.store.robots():
            for kind, color in ((REMOVED, RED), (ADDED, GREEN)):
                records = self.store.records(robot_name, kind)
                box_ns = f"{robot_name}/{kind}_objects"
                label_ns = f"{robot_name}/{kind}_labels"
                box_ids = set()
                label_ids = set()

                for record in records:
                    marker_id = int(record.obj_id & 0x7FFFFFFF)
                    box_ids.add(marker_id)

                    box = Marker()
                    box.header.frame_id = self.target_frame
                    box.header.stamp = zero_stamp
                    box.type = Marker.LINE_LIST
                    box.ns = box_ns
                    box.id = marker_id
                    box.scale.x = self.line_width
                    box.pose.orientation.w = 1.0
                    box.pose.position = Point(
                        x=float(record.center[0]),
                        y=float(record.center[1]),
                        z=float(record.center[2]),
                    )
                    box.color = color
                    box.points = _bbox_line_list_points(record.dimensions)
                    array.markers.append(box)

                    if self.show_labels:
                        label_ids.add(marker_id)
                        text = Marker()
                        text.header.frame_id = self.target_frame
                        text.header.stamp = zero_stamp
                        text.type = Marker.TEXT_VIEW_FACING
                        text.ns = label_ns
                        text.id = marker_id
                        text.scale.z = self.text_scale
                        text.color = color
                        text.pose.orientation.w = 1.0
                        text.pose.position = Point(
                            x=float(record.center[0]),
                            y=float(record.center[1]),
                            z=float(
                                record.center[2]
                                + record.dimensions[2] / 2.0
                                + self.text_scale
                            ),
                        )
                        text.text = self._label_text(record)
                        array.markers.append(text)

                cur_ids[(robot_name, box_ns)] = box_ids
                cur_ids[(robot_name, label_ns)] = label_ids

        # Delete markers whose ids were present last publish but are gone now.
        for key, prev_ids in self._prev_marker_ids.items():
            robot_name, ns = key
            for stale_id in prev_ids - cur_ids.get(key, set()):
                delete_marker = Marker()
                delete_marker.header.frame_id = self.target_frame
                delete_marker.header.stamp = zero_stamp
                delete_marker.ns = ns
                delete_marker.id = stale_id
                delete_marker.action = Marker.DELETE
                array.markers.append(delete_marker)

        self._prev_marker_ids = cur_ids
        self.marker_pub.publish(array)


def _quat_trans_to_matrix(x, y, z, w, tx, ty, tz) -> np.ndarray:
    mat = np.eye(4)
    mat[:3, :3] = _quat_to_rotmat(x, y, z, w)
    mat[:3, 3] = [tx, ty, tz]
    return mat


def _quat_to_rotmat(x, y, z, w) -> np.ndarray:
    n = x * x + y * y + z * z + w * w
    if n < 1e-12:
        return np.eye(3)
    s = 2.0 / n
    xs, ys, zs = x * s, y * s, z * s
    wx, wy, wz = w * xs, w * ys, w * zs
    xx, xy, xz = x * xs, x * ys, x * zs
    yy, yz, zz = y * ys, y * zs, z * zs
    return np.array(
        [
            [1.0 - (yy + zz), xy - wz, xz + wy],
            [xy + wz, 1.0 - (xx + zz), yz - wx],
            [xz - wy, yz + wx, 1.0 - (xx + yy)],
        ]
    )


def _transform_point(mat: np.ndarray, point: np.ndarray) -> np.ndarray:
    return mat[:3, :3] @ point + mat[:3, 3]


def main(args=None):
    rclpy.init(args=args)
    node = AwcdVisualizerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
