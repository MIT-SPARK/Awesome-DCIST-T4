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
the same change merged into one `ChangeRecord` with namespace `RECONCILED` and the merged reports
in `members`. They are recomputed by `reconcile()`, which the owner calls once after updating the
robots' slices, from the latest per-robot snapshots (no state of their own), independent of who
reads them. First the configured `ChangeFilter`s drop records (e.g. added objects that re-detect a
prior object, ignored labels); then removed objects merge by prior-DSG NodeId (shared by all
robots; any robot reporting it removed keeps it removed) and added objects by world-AABB
containment with matching labels. Added records of the same robot merge too: a robot that revisits
an area reports an object it already reported under a new track id.

Limitation: all overlap tests (merging and filters) only handle axis-aligned boxes. An oriented box
is replaced by the world AABB enclosing it, which overestimates its volume and overlap when it is
rotated (e.g. a long object at 45 degrees).
"""

import zlib
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Set, Tuple

import numpy as np

ADDED = "added"
REMOVED = "removed"
# Namespace of reconciled (merged) records.
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
    # Reconciled records only: the merged reports (possibly several from one robot).
    members: List["ChangeRecord"] = field(default_factory=list)


@dataclass
class PriorObject:
    """An object of the prior DSG, with its box in the target frame (for the change filters)."""

    obj_id: int  # prior-DSG NodeId
    center: np.ndarray  # (3,) float64
    dimensions: np.ndarray  # (3,) float64
    orientation: np.ndarray  # (4,) float64 xyzw quaternion
    semantic_label: int


@dataclass
class ReconcileConfig:
    """Merge criteria of `AwcdChangeStore.reconciled()`."""

    # Only merge added objects whose labels match (unknown labels, < 0, match anything).
    require_same_label: bool = True
    # Min overlap of the two world AABBs, as a fraction of the smaller box's volume.
    merge_containment_threshold: float = 0.5
    # Box dimensions are clamped to at least this [m] so flat boxes don't have ~zero volume.
    min_box_dimension: float = 0.05


def reconciled_added_id(namespace: str, track_id: int) -> int:
    """Deterministic int64 id for an added object; track ids alone collide across robots."""
    return (_namespace_crc(namespace) << 32) | (track_id & 0xFFFFFFFF)


def _namespace_crc(namespace: str) -> int:
    return zlib.crc32(namespace.encode()) & 0x7FFFFFFF


def _world_aabbs(objects, min_dimension: float) -> Tuple[np.ndarray, np.ndarray]:
    """(N, 3) min and max corners of the axis-aligned boxes enclosing the objects' oriented boxes.

    Only axis-aligned overlap is supported: rotated boxes are approximated by this enclosing AABB.
    """
    if not objects:
        return np.zeros((0, 3)), np.zeros((0, 3))
    centers = np.array([o.center for o in objects], dtype=float)
    half = (
        np.maximum(
            np.array([o.dimensions for o in objects], dtype=float), min_dimension
        )
        / 2.0
    )
    rotations = np.abs(np.array([_quat_to_rotmat(o.orientation) for o in objects]))
    extents = np.einsum("nij,nj->ni", rotations, half)
    return centers - extents, centers + extents


def _overlap_volumes(a, b) -> np.ndarray:
    """(Na, Nb) overlap volumes of two sets of AABBs given as (min, max) corner arrays."""
    overlap = np.minimum(a[1][:, None], b[1][None]) - np.maximum(
        a[0][:, None], b[0][None]
    )
    return np.prod(np.clip(overlap, 0.0, None), axis=2)


def _volumes(boxes) -> np.ndarray:
    return np.prod(boxes[1] - boxes[0], axis=1)


def _labels_match(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    """(Na, Nb) label compatibility: equal, or either unknown (< 0)."""
    return (a[:, None] == b[None]) | (a[:, None] < 0) | (b[None] < 0)


@dataclass
class FilterContext:
    """What the change filters may look at besides the record itself."""

    prior_objects: List[PriorObject]
    # Prior NodeIds that some robot reports removed (before any filtering).
    removed_ids: Set[int]


class ChangeFilter:
    """Drops records before merging. `keep()` returns a mask over `records` (all of one kind)."""

    def keep(
        self, kind: str, records: List[ChangeRecord], context: FilterContext
    ) -> np.ndarray:
        raise NotImplementedError


class PriorObjectOverlapFilter(ChangeFilter):
    """Drops added objects that lie mostly inside a non-removed prior object of the same label.

    Such a report is usually the prior object itself, re-detected with localization error. With
    require_same_label, objects placed on or in other prior objects (a box on a table) survive.
    """

    @dataclass
    class Config:
        # Fraction of the added box's volume inside the prior object's box to drop it.
        max_overlap_ratio: float = 0.5
        # Only prior objects of the same label (unknown matches anything) count.
        require_same_label: bool = True
        min_box_dimension: float = 0.05

    def __init__(self, config: Optional["PriorObjectOverlapFilter.Config"] = None):
        self.config = config or PriorObjectOverlapFilter.Config()

    def keep(self, kind, records, context):
        keep = np.ones(len(records), dtype=bool)
        priors = [
            p for p in context.prior_objects if p.obj_id not in context.removed_ids
        ]
        if kind != ADDED or not records or not priors:
            return keep
        added = _world_aabbs(records, self.config.min_box_dimension)
        prior = _world_aabbs(priors, self.config.min_box_dimension)
        fraction = _overlap_volumes(added, prior) / _volumes(added)[:, None]
        hits = fraction >= self.config.max_overlap_ratio
        if self.config.require_same_label:
            hits &= _labels_match(
                np.array([r.semantic_label for r in records]),
                np.array([p.semantic_label for p in priors]),
            )
        return ~hits.any(axis=1)


class LabelDropFilter(ChangeFilter):
    """Drops added / removed records whose label is listed (e.g. added chairs, removed trash cans)."""

    @dataclass
    class Config:
        # Integer category ids of the deployed label space.
        added_labels: List[int] = field(default_factory=list)
        removed_labels: List[int] = field(default_factory=list)

    def __init__(self, config: Optional["LabelDropFilter.Config"] = None):
        self.config = config or LabelDropFilter.Config()

    def keep(self, kind, records, context):
        dropped = set(
            self.config.added_labels if kind == ADDED else self.config.removed_labels
        )
        return np.array([r.semantic_label not in dropped for r in records], dtype=bool)


# Filter type name (the `type` of a configured filter) -> filter class; each has a nested Config.
FILTER_TYPES = {
    "PriorObjectOverlapFilter": PriorObjectOverlapFilter,
    "LabelDropFilter": LabelDropFilter,
}


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

    def __init__(
        self,
        config: Optional[ReconcileConfig] = None,
        filters: Optional[List[ChangeFilter]] = None,
    ) -> None:
        self.config = config or ReconcileConfig()
        # Applied in order to the per-robot records by reconcile(), before merging.
        self.filters: List[ChangeFilter] = list(filters or [])
        self._prior_objects: List[PriorObject] = []
        self._slices: Dict[Tuple[str, str], List[ChangeRecord]] = {}
        # reconciled_added_id() crc -> robot name, to catch two robots whose ids would collide.
        self._namespace_crcs: Dict[int, str] = {}
        # Merge of all slices, as of the last reconcile().
        self._reconciled: List[ChangeRecord] = []

    def update(self, namespace: str, kind: str, records: List[ChangeRecord]) -> None:
        """Replace the (namespace, kind) slice of the store with `records`.

        The incoming message is authoritative -- the robot already republishes its complete,
        refined current change set, so no per-id state is carried over from the previous
        message. Raises ValueError if the robot's name collides with another robot's in the
        reconciled added-object ids (same 31-bit crc32).
        """
        owner = self._namespace_crcs.setdefault(_namespace_crc(namespace), namespace)
        if owner != namespace:
            raise ValueError(
                f"Robot names '{owner}' and '{namespace}' collide in reconciled added-object ids"
            )
        self._slices[(namespace, kind)] = list(records)

    def set_prior_objects(self, objects: List[PriorObject]) -> None:
        """Prior-DSG objects (in the target frame) for the change filters."""
        self._prior_objects = list(objects)

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

        Call once after updating the robots' slices. The filters drop records first. Removed
        objects then merge by NodeId. Added objects merge greedily, in order of decreasing AABB
        containment, as long as a group keeps one label; records of the same robot merge too.
        """
        order = lambda r: (r.namespace, r.obj_id)  # noqa: E731
        # Removed ids before filtering: a removed object whose report is dropped (e.g. by label) is
        # still gone, so it must not make overlapping added objects look like re-detections.
        context = FilterContext(
            prior_objects=self._prior_objects,
            removed_ids={r.obj_id for r in self.records(kind=REMOVED)},
        )
        removed = self._filter(
            REMOVED, sorted(self.records(kind=REMOVED), key=order), context
        )
        added = self._filter(
            ADDED, sorted(self.records(kind=ADDED), key=order), context
        )

        removed_groups: Dict[int, List[ChangeRecord]] = {}
        for record in removed:
            removed_groups.setdefault(record.obj_id, []).append(record)
        result = [
            _fuse(REMOVED, obj_id, members)
            for obj_id, members in sorted(removed_groups.items())
        ]

        for members in self._group_added(added, self.config):
            first = min(members, key=lambda r: (r.stamp_ns, r.namespace, r.obj_id))
            obj_id = reconciled_added_id(first.namespace, first.obj_id)
            result.append(_fuse(ADDED, obj_id, members))
        self._reconciled = result

    def _filter(
        self, kind: str, records: List[ChangeRecord], context: FilterContext
    ) -> List[ChangeRecord]:
        for change_filter in self.filters:
            if not records:
                break
            mask = change_filter.keep(kind, records, context)
            records = [r for r, keep in zip(records, mask) if keep]
        return records

    @staticmethod
    def _group_added(
        records: List[ChangeRecord], config: ReconcileConfig
    ) -> List[List[ChangeRecord]]:
        boxes = _world_aabbs(records, config.min_box_dimension)
        volumes = _volumes(boxes)
        smaller = np.minimum(volumes[:, None], volumes[None])
        overlap = _overlap_volumes(boxes, boxes)
        containment = np.divide(
            overlap, smaller, out=np.zeros_like(overlap), where=smaller > 0.0
        )
        # Candidate pairs i < j, strongest containment first (ties by index, deterministic).
        rows, cols = np.nonzero(
            np.triu(containment >= config.merge_containment_threshold, k=1)
        )
        order = np.lexsort((cols, rows, -containment[rows, cols]))
        pairs = list(zip(rows[order].tolist(), cols[order].tolist()))

        # Union-find; each root tracks the known labels in its group.
        parent = list(range(len(records)))
        labels = [
            {r.semantic_label} if r.semantic_label >= 0 else set() for r in records
        ]

        def find(i):
            while parent[i] != i:
                parent[i] = parent[parent[i]]
                i = parent[i]
            return i

        for i, j in pairs:
            ri, rj = find(i), find(j)
            if ri == rj:
                continue
            if config.require_same_label and labels[ri] and labels[rj]:
                if labels[ri] != labels[rj]:
                    continue
            parent[rj] = ri
            labels[ri] |= labels[rj]

        groups: Dict[int, List[ChangeRecord]] = {}
        for i, record in enumerate(records):
            groups.setdefault(find(i), []).append(record)
        return list(groups.values())
