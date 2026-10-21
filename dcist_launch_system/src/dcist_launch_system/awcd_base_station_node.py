#!/usr/bin/env python3
"""Base-station node for Active Window Change Detector (AWCD) reports.

Subscribes to the base station's own prior DSG (published by prior_dsg_publisher_node) and to the
global `khronos_msgs/AwcdChanges` topic that every robot's `ActiveWindowChangeDetectorPublisher`
publishes to. It keeps the change memory in an `AwcdChangeStore` (awcd_change_store.py): each
robot's latest change set, transformed into `target_frame`, plus the cross-robot reconciled changes.

After every robot message, the store is handed to the node's outputs:
- `ReconciledChangesPublisher`: sends the reconciled changes downstream. Today this is a latched
  `AwcdChanges` (robot_name "reconciled") on `~/reconciled_changes`; other downstream protocols are
  added as further outputs with a `call(store)` method.
- `AwcdVisualizer` (awcd_visualizer.py, `visualization.enabled`): RViz markers for the operator.
"""

import time
from typing import Dict

import numpy as np
import rclpy
import tf2_ros
from geometry_msgs.msg import Point
from hydra_ros import DsgSubscriber
from khronos_msgs.msg import AwcdChanges, ChangedObjectInfo
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time

from dcist_launch_system.awcd_change_store import (
    ADDED,
    RECONCILED,
    REMOVED,
    AwcdChangeStore,
    ChangeRecord,
    ReconcileConfig,
)
from dcist_launch_system.awcd_visualizer import AwcdVisualizer


class ReconciledChangesPublisher:
    """Downstream output: publishes the reconciled changes whenever their membership changes."""

    def __init__(self, node: Node, frame_id: str):
        self.frame_id = frame_id
        self.clock = node.get_clock()
        latched = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self.pub = node.create_publisher(AwcdChanges, "~/reconciled_changes", latched)
        # (kind, id, members) of the last published message, for change-only publishing.
        self._last_key = None

    def call(self, store: AwcdChangeStore):
        records = store.reconciled()
        key = sorted(
            (r.kind, r.obj_id, tuple((m.namespace, m.obj_id) for m in r.members))
            for r in records
        )
        if key == self._last_key:
            return
        self._last_key = key

        msg = AwcdChanges()
        msg.header.stamp = self.clock.now().to_msg()
        msg.header.frame_id = self.frame_id
        msg.robot_name = RECONCILED
        for record in records:
            info = _make_changed_object_info(record)
            if record.kind == REMOVED:
                msg.removed_objects.append(info)
            else:
                msg.added_objects.append(info)
        self.pub.publish(msg)


class AwcdBaseStationNode(Node):
    def __init__(self):
        super().__init__("awcd_base_station")

        self.target_frame = self.declare_parameter("target_frame", "map").value
        self.tf_fallback_identity = self.declare_parameter(
            "tf_fallback_identity", True
        ).value
        self.store = AwcdChangeStore(
            ReconcileConfig(
                require_same_label=self.declare_parameter(
                    "require_same_label", True
                ).value,
                merge_containment_threshold=self.declare_parameter(
                    "merge_containment_threshold", 0.5
                ).value,
            )
        )

        self.graph = None
        # Latest raw message per robot, so we can re-ingest (rebuild removed records) once the
        # prior DSG arrives, without waiting for the robot's next (change-gated) publish.
        self._latest_msgs: Dict[str, AwcdChanges] = {}

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Everything that consumes the change memory; each has a call(store) method.
        self.outputs = [ReconciledChangesPublisher(self, self.target_frame)]
        if self.declare_parameter("visualization.enabled", True).value:
            self.outputs.append(AwcdVisualizer(self, self.target_frame))

        self.dsg_sub = DsgSubscriber(self, "~/dsg_in", self._on_dsg)
        self.changes_sub = self.create_subscription(
            AwcdChanges, "/awcd_changes", self._on_changes, 10
        )

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
        self.store.reconcile()

        for output in self.outputs:
            output.call(self.store)

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
            # RAABB/OBB boxes carry a rotation; AABB boxes are axis-aligned in the prior frame.
            world_R_center = (
                np.array(bbox.world_R_center, dtype=float)
                if bbox.has_rotation()
                else np.eye(3)
            )
            orientation = _rotmat_to_quat(transform[:3, :3] @ world_R_center)
            semantic_label = getattr(attrs, "semantic_label", -1)
            records.append(
                ChangeRecord(
                    namespace=msg.robot_name,
                    kind=REMOVED,
                    obj_id=int(info.id),
                    stamp_ns=Time.from_msg(info.stamp).nanoseconds,
                    center=center,
                    dimensions=dims,
                    orientation=orientation,
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
            msg_R_center = _quat_to_rotmat(
                info.bbox_orientation.x,
                info.bbox_orientation.y,
                info.bbox_orientation.z,
                info.bbox_orientation.w,
            )
            orientation = _rotmat_to_quat(transform[:3, :3] @ msg_R_center)
            records.append(
                ChangeRecord(
                    namespace=msg.robot_name,
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


def _make_changed_object_info(record: ChangeRecord) -> ChangedObjectInfo:
    """Fill a ChangedObjectInfo, with the bbox for both kinds (all in the target frame)."""
    info = ChangedObjectInfo()
    info.id = int(record.obj_id)
    info.stamp = Time(nanoseconds=int(record.stamp_ns)).to_msg()
    info.change_confidence = float(record.change_confidence)
    info.num_frames_observed = int(record.num_frames_observed)
    center = Point(
        x=float(record.center[0]), y=float(record.center[1]), z=float(record.center[2])
    )
    info.centroid = center
    info.bbox_center = center
    info.bbox_dimensions.x = float(record.dimensions[0])
    info.bbox_dimensions.y = float(record.dimensions[1])
    info.bbox_dimensions.z = float(record.dimensions[2])
    info.bbox_orientation.x = float(record.orientation[0])
    info.bbox_orientation.y = float(record.orientation[1])
    info.bbox_orientation.z = float(record.orientation[2])
    info.bbox_orientation.w = float(record.orientation[3])
    info.semantic_label = int(record.semantic_label)
    info.confidence = float(record.confidence)
    return info


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


def _rotmat_to_quat(rot: np.ndarray) -> np.ndarray:
    """Rotation matrix to quaternion [x, y, z, w]."""
    trace = rot[0, 0] + rot[1, 1] + rot[2, 2]
    if trace > 0.0:
        s = 2.0 * np.sqrt(trace + 1.0)
        w = 0.25 * s
        x = (rot[2, 1] - rot[1, 2]) / s
        y = (rot[0, 2] - rot[2, 0]) / s
        z = (rot[1, 0] - rot[0, 1]) / s
    elif rot[0, 0] > rot[1, 1] and rot[0, 0] > rot[2, 2]:
        s = 2.0 * np.sqrt(1.0 + rot[0, 0] - rot[1, 1] - rot[2, 2])
        w = (rot[2, 1] - rot[1, 2]) / s
        x = 0.25 * s
        y = (rot[0, 1] + rot[1, 0]) / s
        z = (rot[0, 2] + rot[2, 0]) / s
    elif rot[1, 1] > rot[2, 2]:
        s = 2.0 * np.sqrt(1.0 + rot[1, 1] - rot[0, 0] - rot[2, 2])
        w = (rot[0, 2] - rot[2, 0]) / s
        x = (rot[0, 1] + rot[1, 0]) / s
        y = 0.25 * s
        z = (rot[1, 2] + rot[2, 1]) / s
    else:
        s = 2.0 * np.sqrt(1.0 + rot[2, 2] - rot[0, 0] - rot[1, 1])
        w = (rot[1, 0] - rot[0, 1]) / s
        x = (rot[0, 2] + rot[2, 0]) / s
        y = (rot[1, 2] + rot[2, 1]) / s
        z = 0.25 * s
    quat = np.array([x, y, z, w])
    return quat / np.linalg.norm(quat)


def _transform_point(mat: np.ndarray, point: np.ndarray) -> np.ndarray:
    return mat[:3, :3] @ point + mat[:3, 3]


def main(args=None):
    rclpy.init(args=args)
    node = AwcdBaseStationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
