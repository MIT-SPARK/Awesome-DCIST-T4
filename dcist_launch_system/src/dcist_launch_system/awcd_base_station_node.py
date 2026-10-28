#!/usr/bin/env python3
"""Base-station node for Active Window Change Detector (AWCD) reports.

Subscribes to the base station's own prior DSG (published by prior_dsg_publisher_node) and to the
global `khronos_msgs/AwcdChanges` topic that every robot's `ActiveWindowChangeDetectorPublisher`
publishes to. It keeps the change memory in an `AwcdChangeStore` (awcd_change_store.py): each
robot's latest change set, transformed into `target_frame`, plus the reconciled changes (filtered
by the configured `filters`, then merged).

The ROS callbacks only keep each robot's latest message (and the latest prior DSG). A worker thread
merges on a snapshot of them and, if anything arrived meanwhile, merges again on the newest
snapshot, so it never falls behind on outdated messages. After every merge, the store is handed to
the node's outputs:
- `ReconciledChangesPublisher`: sends the reconciled changes downstream. Today this is a latched
  `AwcdChanges` (robot_name "reconciled") on `~/reconciled_changes`; other downstream protocols are
  added as further outputs with a `call(store)` method.
- `AwcdVisualizer` (awcd_visualizer.py, `visualization.enabled`): RViz markers for the operator.
"""

import dataclasses
import threading
import time
from typing import Dict

import numpy as np
import rclpy
import spark_dsg
import tf2_ros
from geometry_msgs.msg import Point
from hydra_ros import DsgSubscriber
from khronos_msgs.msg import AwcdChanges, ChangedObjectInfo
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time

from dcist_launch_system.awcd_change_store import (
    ADDED,
    FILTER_TYPES,
    RECONCILED,
    REMOVED,
    AwcdChangeStore,
    ChangeRecord,
    PriorObject,
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
        # (kind, id, members, box) of the last published message, for change-only publishing.
        # Boxes are compared exactly: they are recomputed deterministically from the messages.
        self._last_key = None

    def call(self, store: AwcdChangeStore):
        records = store.reconciled()
        key = sorted(
            (
                r.kind,
                r.obj_id,
                tuple((m.namespace, m.obj_id) for m in r.members),
                tuple(r.center.tolist()),
                tuple(r.dimensions.tolist()),
                tuple(r.orientation.tolist()),
            )
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
            ),
            filters=self._make_filters(),
        )

        # Shared with the callbacks, guarded by _lock: latest raw message per robot and prior DSG.
        # Raw messages let the worker rebuild removed records once the prior DSG arrives.
        self._lock = threading.Lock()
        self._latest_msgs: Dict[str, AwcdChanges] = {}
        self._latest_graph = None
        self._pending = threading.Event()
        self._stopped = threading.Event()
        # Worker-only state: the prior DSG in use, its objects' signature, and the message each
        # robot was last ingested from.
        self.graph = None
        self._prior_signature = None
        self._ingested_msgs: Dict[str, AwcdChanges] = {}

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
        self._worker = threading.Thread(target=self._run, daemon=True)
        self._worker.start()

    def destroy_node(self):
        self._stopped.set()
        self._pending.set()
        self._worker.join(timeout=2.0)
        super().destroy_node()

    def _make_filters(self):
        """Filters from `filters` (names, in order); each name's `type` and fields are under it."""
        names = (
            self.declare_parameter("filters", Parameter.Type.STRING_ARRAY).value or []
        )
        filters = []
        for name in names:
            type_name = self.declare_parameter(
                f"{name}.type", Parameter.Type.STRING
            ).value
            filter_cls = FILTER_TYPES.get(type_name)
            if filter_cls is None:
                self.get_logger().error(
                    f"Filter '{name}': unknown type '{type_name}' (known: {sorted(FILTER_TYPES)}); "
                    "skipping."
                )
                continue
            config = self._declare_config(name, filter_cls.Config)
            self.get_logger().info(f"Filter '{name}': {type_name} {config}")
            filters.append(filter_cls(config))
        return filters

    def _declare_config(self, prefix: str, config_cls):
        """Build a dataclass config from parameters `<prefix>.<field>`; unset fields keep defaults.

        ROS 2 parameters can't hold lists of maps, so each filter's fields live under its name.
        List fields are integer arrays declared without a default (a YAML `[]` has no type).
        """
        values = {}
        for f in dataclasses.fields(config_cls):
            name = f"{prefix}.{f.name}"
            if f.default_factory is not dataclasses.MISSING:
                value = self.declare_parameter(name, Parameter.Type.INTEGER_ARRAY).value
                if value is not None:
                    values[f.name] = list(value)
            else:
                values[f.name] = self.declare_parameter(name, f.default).value
        return config_cls(**values)

    def _on_dsg(self, header, graph):
        with self._lock:
            self._latest_graph = graph
        self._pending.set()

    def _on_changes(self, msg: AwcdChanges):
        with self._lock:
            self._latest_msgs[msg.robot_name] = msg
        self._pending.set()

    def _run(self):
        """Merge on the newest snapshot until stopped; reruns at once if something arrived."""
        while not self._stopped.is_set():
            if not self._pending.wait(timeout=0.5):
                continue
            with self._lock:
                self._pending.clear()
                msgs = dict(self._latest_msgs)
                graph = self._latest_graph
            if self._stopped.is_set():
                return
            try:
                self._merge(msgs, graph)
            except Exception as exc:  # keep the worker alive on a bad message
                self.get_logger().error(f"AWCD merge failed: {exc!r}")

    def _merge(self, msgs: Dict[str, AwcdChanges], graph):
        changed = False
        if graph is not self.graph:
            # The prior publisher repeats the same file; only a different prior counts as a change.
            self.graph = graph
            prior_objects = _prior_objects(graph)
            signature = _prior_signature(prior_objects)
            if signature != self._prior_signature:
                self._prior_signature = signature
                self.store.set_prior_objects(prior_objects)
                # Removed records resolve their bboxes in the prior DSG: re-ingest every robot.
                self._ingested_msgs.clear()
                changed = True

        for robot, msg in msgs.items():
            if self._ingested_msgs.get(robot) is not msg and self._ingest(msg):
                self._ingested_msgs[robot] = msg
                changed = True

        if not changed:
            return
        self.store.reconcile()
        for output in self.outputs:
            output.call(self.store)

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

    def _ingest(self, msg: AwcdChanges) -> bool:
        """Replace the robot's slices with the message's records; False if it was skipped."""
        transform = self._lookup_prior_T_msg(msg.header.frame_id)
        if transform is None:
            return False

        removed_records = self._build_removed_records(msg, transform)
        # Added-object attributes are self-contained in the message (no prior-DSG lookup needed),
        # so this never depends on graph availability.
        added_records = self._build_added_records(msg, transform)

        # The message is authoritative for this robot -- always replace both slices, including
        # with an empty removed list when the prior DSG isn't available yet, so no stale removed
        # objects from an earlier message linger indefinitely.
        try:
            self.store.update(msg.robot_name, REMOVED, removed_records)
            self.store.update(msg.robot_name, ADDED, added_records)
        except ValueError as exc:
            self.get_logger().error(
                f"{exc}; ignoring '{msg.robot_name}'.", throttle_duration_sec=5.0
            )
            return False
        return True

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
            orientation = _rotmat_to_quat(transform[:3, :3] @ _bbox_rotation(bbox))
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


def _bbox_rotation(bbox) -> np.ndarray:
    """RAABB/OBB boxes carry a rotation; AABB boxes are axis-aligned in the prior frame."""
    if bbox.has_rotation():
        return np.array(bbox.world_R_center, dtype=float)
    return np.eye(3)


def _prior_objects(graph):
    """The prior DSG's objects with valid boxes; the prior DSG is in the target frame."""
    if graph is None or not graph.has_layer(spark_dsg.DsgLayers.OBJECTS):
        return []
    objects = []
    for node in graph.get_layer(spark_dsg.DsgLayers.OBJECTS).nodes:
        attrs = node.attributes
        bbox = getattr(attrs, "bounding_box", None)
        if bbox is None or not bbox.is_valid():
            continue
        objects.append(
            PriorObject(
                obj_id=int(node.id.value),
                center=np.array(bbox.world_P_center, dtype=float),
                dimensions=np.array(bbox.dimensions, dtype=float),
                orientation=_rotmat_to_quat(_bbox_rotation(bbox)),
                semantic_label=int(getattr(attrs, "semantic_label", -1)),
            )
        )
    return objects


def _prior_signature(objects):
    """Exact identity of the prior objects; ids alone collide across hydra maps (O0, O1, ...)."""
    return tuple(
        (
            o.obj_id,
            o.semantic_label,
            tuple(o.center.tolist()),
            tuple(o.dimensions.tolist()),
            tuple(o.orientation.tolist()),
        )
        for o in sorted(objects, key=lambda o: o.obj_id)
    )


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
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
