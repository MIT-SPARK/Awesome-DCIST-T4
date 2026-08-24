"""In-memory, ROS-free store of per-robot AWCD (Active Window Change Detector) reports.

This module intentionally has no ROS dependencies so it can be unit tested directly and
reused outside of `awcd_visualizer_node.py`.

Each robot's `ActiveWindowChangeDetectorPublisher` publishes its *complete, already-refined*
current change set on every message -- fragments merged, stale records pruned -- so this store
is a latest-snapshot cache, not an accumulator: `AwcdChangeStore.update()` simply replaces the
(robot_name, kind) slice wholesale with the incoming message's records. Keeping any state here
across messages (e.g. re-latching timestamps, merging ids across messages) would just duplicate
authority the robot already has and risks resurrecting objects the robot has since reconciled
away.

The store keys records by `(robot_name, kind, obj_id)` rather than flattening everything
together. This is deliberate: it keeps track of *which robot reported which change*, which is
what lets future work (cross-robot deduplication of the same physical object, confidence-
weighted merging, requiring N-robot consensus before trusting a detection, etc.) build on top
of this store without needing to re-plumb the subscription/rendering code. None of that
filtering/merging logic is implemented here yet -- this is purely the latest-snapshot layer.
"""

from dataclasses import dataclass
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
    stamp_ns: (
        int  # first_removed_ns / first_seen, from the detector (latched, sensor time)
    )
    center: np.ndarray  # (3,) float64, already transformed into the render/target frame
    dimensions: np.ndarray  # (3,) float64
    orientation: np.ndarray  # (4,) float64 xyzw quaternion (not currently rendered)
    semantic_label: int
    confidence: float  # tracking-quality (added-only); not currently rendered
    change_confidence: (
        float  # change-detection EMA that drives the added/removed decision
    )
    num_frames_observed: int  # frames that fed change_confidence
    last_seen_ns: (
        int  # wall-clock ns of the most recent message that reported this record
    )


class AwcdChangeStore:
    """Latest-snapshot store of ChangeRecords per robot, keyed by (robot_name, kind)."""

    def __init__(self) -> None:
        self._slices: Dict[Tuple[str, str], List[ChangeRecord]] = {}

    def update(self, robot_name: str, kind: str, records: List[ChangeRecord]) -> None:
        """Replace the (robot_name, kind) slice of the store with `records`.

        The incoming message is authoritative -- the robot already republishes its complete,
        refined current change set, so no per-id state is carried over from the previous
        message.
        """
        self._slices[(robot_name, kind)] = list(records)

    def robots(self) -> List[str]:
        return sorted({key[0] for key in self._slices})

    def records(
        self, robot_name: Optional[str] = None, kind: Optional[str] = None
    ) -> List[ChangeRecord]:
        return [
            r
            for key, slice_records in self._slices.items()
            if (robot_name is None or key[0] == robot_name)
            and (kind is None or key[1] == kind)
            for r in slice_records
        ]

    def drop_robot(self, robot_name: str) -> None:
        for key in [k for k in self._slices if k[0] == robot_name]:
            del self._slices[key]
