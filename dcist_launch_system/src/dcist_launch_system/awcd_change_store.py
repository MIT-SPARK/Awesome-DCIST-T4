"""In-memory, ROS-free store of per-robot AWCD (Active Window Change Detector) reports.

This module intentionally has no ROS dependencies so it can be unit tested directly. It is the
change memory of `awcd_base_station_node.py`.

Each robot's `ActiveWindowChangeDetectorPublisher` publishes its *complete, already-refined*
current change set on every message -- fragments merged, stale records pruned -- so this store
is a latest-snapshot cache, not an accumulator: `AwcdChangeStore.update()` simply replaces the
(namespace, kind) slice wholesale with the incoming message's records. Keeping any state here
across messages (e.g. re-latching timestamps, merging ids across messages) would just duplicate
authority the robot already has and risks resurrecting objects the robot has since reconciled
away.

The store keys records by `(namespace, kind, obj_id)` (namespace = robot name) rather than flattening everything
together. This is deliberate: it keeps track of *which robot reported which change*, which is
what lets future work (cross-robot deduplication of the same physical object, confidence-
weighted merging, requiring N-robot consensus before trusting a detection, etc.) build on top
of this store without needing to re-plumb the subscription/rendering code.

On top of the per-robot slices, the store holds the reconciled changes (`reconciled()`): reports of
the same change from different robots merged into one `ChangeRecord` with namespace `RECONCILED`
and the merged reports in `members`. They are recomputed by `reconcile()`, which the owner calls once
after updating a robot's slices, from the latest per-robot snapshots (no state of their own),
independent of who reads them: removed objects merge by prior-DSG NodeId (shared
by all robots), added objects by world-AABB containment. Records from the same robot are never
merged (that is the robot's tracker's job).
"""

import zlib
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple

import numpy as np

ADDED = "added"
REMOVED = "removed"
# Namespace of reconciled (cross-robot merged) records.
RECONCILED = "reconciled"


@dataclass
class ChangeRecord:
    """An added/removed object report from one robot, or the merge of several (RECONCILED)."""

    namespace: str  # reporting robot's name, or RECONCILED
    kind: str  # ADDED or REMOVED
    # Prior-DSG NodeId (removed), Track id (added), or reconciled_added_id() (reconciled added).
    obj_id: int
    stamp_ns: (
        int  # first_removed_ns / first_seen, from the detector (latched, sensor time)
    )
    center: np.ndarray  # (3,) float64, already transformed into the render/target frame
    dimensions: np.ndarray  # (3,) float64
    orientation: np.ndarray  # (4,) float64 xyzw quaternion, in the target frame
    semantic_label: int
    confidence: float  # tracking-quality (added-only); not currently rendered
    change_confidence: (
        float  # change-detection EMA that drives the added/removed decision
    )
    num_frames_observed: int  # frames that fed change_confidence
    last_seen_ns: (
        int  # wall-clock ns of the most recent message that reported this record
    )
    # Reconciled records only: the merged per-robot reports, at most one per robot.
    members: List["ChangeRecord"] = field(default_factory=list)


@dataclass
class ReconcileConfig:
    """Cross-robot merge criteria of `AwcdChangeStore.reconciled()`."""

    # Only merge added objects whose labels match (unknown labels, < 0, match anything).
    require_same_label: bool = True
    # Min overlap of the two world AABBs, as a fraction of the smaller box's volume.
    merge_containment_threshold: float = 0.5
    # Box dimensions are clamped to at least this [m] so flat boxes don't have ~zero volume.
    min_box_dimension: float = 0.05


def reconciled_added_id(namespace: str, track_id: int) -> int:
    """Deterministic int64 id for an added object; track ids alone collide across robots."""
    return ((zlib.crc32(namespace.encode()) & 0x7FFFFFFF) << 32) | (
        track_id & 0xFFFFFFFF
    )


def _world_aabb(record: ChangeRecord, min_dimension: float):
    """Min/max corners of the axis-aligned box enclosing the record's oriented box."""
    half = np.maximum(record.dimensions, min_dimension) / 2.0
    extent = np.abs(_quat_to_rotmat(record.orientation)) @ half
    return record.center - extent, record.center + extent


def _containment(a, b) -> float:
    """Overlap volume of two AABBs divided by the smaller AABB's volume."""
    overlap = np.clip(np.minimum(a[1], b[1]) - np.maximum(a[0], b[0]), 0.0, None)
    smaller = min(np.prod(a[1] - a[0]), np.prod(b[1] - b[0]))
    return float(np.prod(overlap) / smaller) if smaller > 0.0 else 0.0


def _quat_to_rotmat(q: np.ndarray) -> np.ndarray:
    x, y, z, w = q / np.linalg.norm(q)
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ]
    )


def _fuse(kind: str, obj_id: int, members: List[ChangeRecord]) -> ChangeRecord:
    """Merged record: earliest stamp, bbox of the best-observed member, max confidences."""
    # members are sorted by (namespace, obj_id), so max() breaks ties deterministically.
    best = max(members, key=lambda r: r.num_frames_observed)
    known_labels = [r.semantic_label for r in members if r.semantic_label >= 0]
    if best.semantic_label >= 0:
        label = best.semantic_label
    else:
        label = known_labels[0] if known_labels else -1
    return ChangeRecord(
        namespace=RECONCILED,
        kind=kind,
        obj_id=obj_id,
        stamp_ns=min(r.stamp_ns for r in members),
        center=best.center,
        dimensions=best.dimensions,
        orientation=best.orientation,
        semantic_label=label,
        confidence=max(r.confidence for r in members),
        change_confidence=max(r.change_confidence for r in members),
        num_frames_observed=sum(r.num_frames_observed for r in members),
        last_seen_ns=max(r.last_seen_ns for r in members),
        members=members,
    )


class AwcdChangeStore:
    """Latest-snapshot store of ChangeRecords per (namespace, kind), plus their reconciliation."""

    def __init__(self, config: Optional[ReconcileConfig] = None) -> None:
        self.config = config or ReconcileConfig()
        self._slices: Dict[Tuple[str, str], List[ChangeRecord]] = {}
        # Cross-robot merge of all slices, as of the last reconcile().
        self._reconciled: List[ChangeRecord] = []

    def update(self, namespace: str, kind: str, records: List[ChangeRecord]) -> None:
        """Replace the (namespace, kind) slice of the store with `records`.

        The incoming message is authoritative -- the robot already republishes its complete,
        refined current change set, so no per-id state is carried over from the previous
        message.
        """
        self._slices[(namespace, kind)] = list(records)

    def robots(self) -> List[str]:
        return sorted({key[0] for key in self._slices})

    def records(
        self, namespace: Optional[str] = None, kind: Optional[str] = None
    ) -> List[ChangeRecord]:
        """Per-robot records, optionally filtered by namespace and kind."""
        return [
            r
            for key, slice_records in self._slices.items()
            if (namespace is None or key[0] == namespace)
            and (kind is None or key[1] == kind)
            for r in slice_records
        ]

    def drop_robot(self, namespace: str) -> None:
        for key in [k for k in self._slices if k[0] == namespace]:
            del self._slices[key]

    def reconciled(self, kind: Optional[str] = None) -> List[ChangeRecord]:
        """Reconciled records (namespace RECONCILED) as of the last reconcile()."""
        return [r for r in self._reconciled if kind is None or r.kind == kind]

    def reconcile(self) -> None:
        """Recompute the RECONCILED records from all robots' latest reports.

        Call once after updating a robot's slices. Removed objects merge by NodeId. Added objects
        merge greedily, in order of decreasing AABB containment, as long as a group keeps at most
        one record per robot and one label.
        """
        order = lambda r: (r.namespace, r.obj_id)  # noqa: E731

        removed_groups: Dict[int, List[ChangeRecord]] = {}
        for record in sorted(self.records(kind=REMOVED), key=order):
            removed_groups.setdefault(record.obj_id, []).append(record)
        result = [
            _fuse(REMOVED, obj_id, members)
            for obj_id, members in sorted(removed_groups.items())
        ]

        added = sorted(self.records(kind=ADDED), key=order)
        for members in self._group_added(added, self.config):
            first = min(members, key=lambda r: (r.stamp_ns, r.namespace, r.obj_id))
            obj_id = reconciled_added_id(first.namespace, first.obj_id)
            result.append(_fuse(ADDED, obj_id, members))
        self._reconciled = result

    @staticmethod
    def _group_added(
        records: List[ChangeRecord], config: ReconcileConfig
    ) -> List[List[ChangeRecord]]:
        boxes = [_world_aabb(r, config.min_box_dimension) for r in records]
        pairs = []
        for i in range(len(records)):
            for j in range(i + 1, len(records)):
                if records[i].namespace == records[j].namespace:
                    continue
                containment = _containment(boxes[i], boxes[j])
                if containment >= config.merge_containment_threshold:
                    pairs.append((-containment, i, j))
        pairs.sort()

        # Union-find; each root tracks the robots and known labels in its group.
        parent = list(range(len(records)))
        robots = [{r.namespace} for r in records]
        labels = [{r.semantic_label} if r.semantic_label >= 0 else set() for r in records]

        def find(i):
            while parent[i] != i:
                parent[i] = parent[parent[i]]
                i = parent[i]
            return i

        for _, i, j in pairs:
            ri, rj = find(i), find(j)
            if ri == rj or robots[ri] & robots[rj]:
                continue
            if config.require_same_label and labels[ri] and labels[rj]:
                if labels[ri] != labels[rj]:
                    continue
            parent[rj] = ri
            robots[ri] |= robots[rj]
            labels[ri] |= labels[rj]

        groups: Dict[int, List[ChangeRecord]] = {}
        for i, record in enumerate(records):
            groups.setdefault(find(i), []).append(record)
        return list(groups.values())
