"""In-memory, ROS-free store of per-robot AWCD (Active Window Change Detector) reports.

This module intentionally has no ROS dependencies so it can be unit tested directly and
reused outside of `awcd_visualizer_node.py`.

Each robot periodically republishes its *entire* lifetime history of added/removed objects
(see `ActiveWindowChangeDetectorPublisher`), so `AwcdChangeStore.update()` treats every
message as authoritative for that robot+kind and replaces the corresponding slice, while
preserving first-observation metadata for records that were already known.

The store keys records by `(robot_name, kind, obj_id)` rather than flattening everything
together. This is deliberate: it keeps track of *which robot reported which change*, which is
what lets future work (cross-robot deduplication of the same physical object, confidence-
weighted merging, requiring N-robot consensus before trusting a detection, etc.) build on top
of this store without needing to re-plumb the subscription/rendering code. None of that
filtering/merging logic is implemented here yet -- this is purely the accumulation layer.
"""

from dataclasses import dataclass, replace
from typing import Dict, List, Optional, Tuple

import numpy as np

ADDED = "added"
REMOVED = "removed"


@dataclass
class ChangeRecord:
    """A single added/removed object report from one robot."""

    robot_name: str
    kind: str  # ADDED or REMOVED
    obj_id: int  # prior-DSG NodeId (removed) or Track id (added)
    stamp_ns: int  # first_removed_ns / first_seen, from the detector (latched, sensor time)
    center: np.ndarray  # (3,) float64, already transformed into the render/target frame
    dimensions: np.ndarray  # (3,) float64
    orientation: np.ndarray  # (4,) float64 xyzw quaternion (not currently rendered)
    semantic_label: int
    confidence: float
    last_seen_ns: int  # wall-clock ns of the most recent message that reported this record


class AwcdChangeStore:
    """Accumulates ChangeRecords across all robots, keyed by (robot_name, kind, obj_id)."""

    def __init__(self) -> None:
        self._records: Dict[Tuple[str, str, int], ChangeRecord] = {}

    def update(self, robot_name: str, kind: str, records: List[ChangeRecord]) -> None:
        """Replace the (robot_name, kind) slice of the store with `records`.

        `stamp_ns` (the latched first-observation time) is preserved from any existing record
        with the same id -- the incoming message's own `stamp_ns` for a previously-known id is
        trusted only when we don't already have one, since the detector already latches it and
        republishing should not change it.
        """
        new_ids = {r.obj_id for r in records}

        # Drop ids for this (robot, kind) that are no longer present in the robot's report.
        for key in [
            k
            for k in self._records
            if k[0] == robot_name and k[1] == kind and k[2] not in new_ids
        ]:
            del self._records[key]

        for record in records:
            key = (robot_name, kind, record.obj_id)
            existing = self._records.get(key)
            if existing is not None:
                record = replace(record, stamp_ns=existing.stamp_ns)
            self._records[key] = record

    def robots(self) -> List[str]:
        return sorted({key[0] for key in self._records})

    def records(
        self, robot_name: Optional[str] = None, kind: Optional[str] = None
    ) -> List[ChangeRecord]:
        return [
            r
            for key, r in self._records.items()
            if (robot_name is None or key[0] == robot_name)
            and (kind is None or key[1] == kind)
        ]

    def drop_robot(self, robot_name: str) -> None:
        for key in [k for k in self._records if k[0] == robot_name]:
            del self._records[key]
