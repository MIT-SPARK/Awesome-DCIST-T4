"""RViz visualization of the AWCD base station's change memory (owned by awcd_base_station_node).

Draws red (removed) / green (added) bounding-box wireframes + text labels -- the base-station
equivalent of `ActiveWindowChangeDetectorVisualizer` (khronos_ros), which only runs on-robot. Two
latched topics give the operator two granularities:
- `~/changed_objects`: each robot's own reports (marker namespaces `<robot>/<kind>_objects`).
- `~/reconciled_objects`: the merged changes (`reconciled/<kind>_objects`).
"""

from typing import Dict, List

import numpy as np
from geometry_msgs.msg import Point
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray

from dcist_launch_system.awcd_change_store import (
    REMOVED,
    AwcdChangeStore,
    ChangeRecord,
)

RED = ColorRGBA(r=1.0, g=0.0, b=0.0, a=1.0)
GREEN = ColorRGBA(r=0.0, g=1.0, b=0.0, a=1.0)


def _bbox_line_list_points(dimensions: np.ndarray) -> List[Point]:
    """12-edge / 24-point wireframe in the box frame, mirroring khronos_ros setBoundingBox().

    The marker pose carries the box center and orientation.
    """
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


class AwcdVisualizer:
    """Publishes the per-robot and reconciled changes of an AwcdChangeStore as RViz markers."""

    def __init__(self, node: Node, target_frame: str):
        self.target_frame = target_frame
        self.line_width = node.declare_parameter("visualization.line_width", 0.1).value
        self.text_scale = node.declare_parameter("visualization.text_scale", 0.4).value
        self.show_labels = node.declare_parameter(
            "visualization.show_labels", True
        ).value
        # TODO(nathan) get this from ROS
        self.labelspace = {}

        # Latched: every message holds the full current layer, so late RViz sessions catch up.
        latched = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self.robot_pub = node.create_publisher(
            MarkerArray, "~/changed_objects", latched
        )
        self.reconciled_pub = node.create_publisher(
            MarkerArray, "~/reconciled_objects", latched
        )
        # Marker ids emitted per marker namespace in the previous publish of each topic, so we
        # can emit DELETEs for ids that dropped out -- python equivalent of
        # MarkerTracker::clearPrevious.
        self._prev_ids: Dict[str, Dict[str, set]] = {"robot": {}, "reconciled": {}}

    def call(self, store: AwcdChangeStore):
        self._publish(self.robot_pub, "robot", store.records())
        self._publish(self.reconciled_pub, "reconciled", store.reconciled())

    def _publish(self, pub, layer: str, records: List[ChangeRecord]):
        array = MarkerArray()
        cur_ids: Dict[str, set] = {}
        for record in records:
            box_ns = f"{record.namespace}/{record.kind}_objects"
            label_ns = f"{record.namespace}/{record.kind}_labels"
            # Reconciled added ids carry a robot hash in the upper 32 bits; fold it into int32.
            marker_id = int((record.obj_id ^ (record.obj_id >> 32)) & 0x7FFFFFFF)
            cur_ids.setdefault(box_ns, set()).add(marker_id)
            if self.show_labels:
                cur_ids.setdefault(label_ns, set()).add(marker_id)
            self._append_record_markers(
                array,
                record,
                box_ns,
                label_ns,
                marker_id,
                RED if record.kind == REMOVED else GREEN,
                self._label_text(record),
            )
        self._append_stale_deletes(array, self._prev_ids[layer], cur_ids)
        self._prev_ids[layer] = cur_ids
        pub.publish(array)

    def _label_text(self, record: ChangeRecord) -> str:
        if record.semantic_label < 0:
            label_name = "unknown"
        else:
            label_name = self.labelspace.get(
                record.semantic_label, f"label_{record.semantic_label}"
            )
        if record.members:
            source = ", ".join(f"{m.namespace}:{m.obj_id}" for m in record.members)
            source += f" | {record.kind}"
        else:
            source = f"{record.namespace} | {record.kind} {record.obj_id}"
        return (
            f"{source} | {label_name} | "
            f"cc={record.change_confidence:.2f} n={record.num_frames_observed}"
        )

    def _append_record_markers(
        self, array, record, box_ns, label_ns, marker_id, color, text
    ):
        """Append the bbox wireframe (and label, if enabled) of a record."""
        zero_stamp = Time().to_msg()
        box = Marker()
        box.header.frame_id = self.target_frame
        box.header.stamp = zero_stamp
        box.type = Marker.LINE_LIST
        box.ns = box_ns
        box.id = marker_id
        box.scale.x = self.line_width
        box.pose.orientation.x = float(record.orientation[0])
        box.pose.orientation.y = float(record.orientation[1])
        box.pose.orientation.z = float(record.orientation[2])
        box.pose.orientation.w = float(record.orientation[3])
        box.pose.position = Point(
            x=float(record.center[0]),
            y=float(record.center[1]),
            z=float(record.center[2]),
        )
        box.color = color
        box.points = _bbox_line_list_points(record.dimensions)
        array.markers.append(box)

        if not self.show_labels:
            return
        label = Marker()
        label.header.frame_id = self.target_frame
        label.header.stamp = zero_stamp
        label.type = Marker.TEXT_VIEW_FACING
        label.ns = label_ns
        label.id = marker_id
        label.scale.z = self.text_scale
        label.color = color
        label.pose.orientation.w = 1.0
        label.pose.position = Point(
            x=float(record.center[0]),
            y=float(record.center[1]),
            z=float(record.center[2] + record.dimensions[2] / 2.0 + self.text_scale),
        )
        label.text = text
        array.markers.append(label)

    def _append_stale_deletes(self, array, prev_ids, cur_ids):
        """DELETE markers whose ids were present last publish but are gone now."""
        zero_stamp = Time().to_msg()
        for ns, ids in prev_ids.items():
            for stale_id in ids - cur_ids.get(ns, set()):
                delete_marker = Marker()
                delete_marker.header.frame_id = self.target_frame
                delete_marker.header.stamp = zero_stamp
                delete_marker.ns = ns
                delete_marker.id = stale_id
                delete_marker.action = Marker.DELETE
                array.markers.append(delete_marker)
