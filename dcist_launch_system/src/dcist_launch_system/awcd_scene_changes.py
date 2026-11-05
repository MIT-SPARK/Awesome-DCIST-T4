"""Turn AWCD reports into scene changes for the scene change writer. No ROS dependencies.

Two ways to send them (the bridge's send_full_change_set parameter):

* full change set (``snapshot``, the default): every report is sent whole, each change with
  AWCD's id for it -- the prior-map symbol of a removed object, "awcd:<robot>:<id>" for an added
  one. The writer keeps the map in step with the set: a change no longer in it was withdrawn
  (e.g. after a change detection reset) and is undone, so a removed object comes back.
* new changes only (``changes``, below): each change is sent once and never undone. This is
  what older writers understand.

The reports are either each robot's own (`ActiveWindowChangeDetectorPublisher`) or the base
station's reconciled set (awcd_base_station_node, robot_name "reconciled"). Both republish their
*complete* current change set whenever it changes (see awcd_change_store.py). The scene change
writer, on the other
hand, applies every change it is sent: an ADDED change sent twice creates the object twice. So
this keeps track of what has already been sent and emits each change once:

* removed objects by their prior-map symbol (e.g. "O16"), across all robots -- two robots seeing
  the same object gone is one removal. The writer deletes the node, which cannot be undone, so an
  object that later drops out of a robot's removed set stays removed.
* added objects by (robot, AWCD object id). The id is AWCD's, not a map symbol, so the change is
  sent with no symbol and the writer assigns the next free one. Its class is the report's name
  when it has one (reconciled reports do), else the robot's category id looked up in its
  labelspace.

AWCD reports no moves: a moved object arrives as one removed and one added object.

Thresholds on change_confidence / num_frames_observed are applied on top of AWCD's own decision.
Because AWCD only republishes when the id set changes, a record below threshold is not re-offered
when its confidence later rises; the defaults (0) therefore leave the decision to AWCD.
"""

from dataclasses import dataclass
from typing import Callable, Dict, List, Optional, Set, Tuple

import spark_dsg

ADDED, REMOVED = 0, 1  # heracles_ros_interfaces/SceneChange kinds


@dataclass
class ReportedObject:
    """One entry of AwcdChanges.removed_objects / added_objects."""

    obj_id: int  # prior-map NodeId (removed) or AWCD object id (added)
    change_confidence: float
    num_frames_observed: int
    semantic_label: int = -1  # added only
    position: Tuple[float, float, float] = (0.0, 0.0, 0.0)  # added only, map frame
    name: str = ""  # added only: the class name, when the report carries it


@dataclass
class Change:
    kind: int  # ADDED or REMOVED
    symbol: str  # prior-map symbol for REMOVED, "" for ADDED
    semantic_label: str
    position: Tuple[float, float, float]
    confidence: float


def removed_symbol(node_id: int) -> str:
    """Prior-map NodeId -> the nodeSymbol Neo4j stores, e.g. "O16"."""
    return spark_dsg.NodeSymbol(int(node_id)).str(True)


class AwcdSceneChanges:
    def __init__(self, min_change_confidence: float = 0.0, min_frames: int = 0) -> None:
        self.min_change_confidence = min_change_confidence
        self.min_frames = min_frames
        self._removed_sent: Set[str] = set()
        self._added_sent: Set[Tuple[str, int]] = set()

    def _passes(self, obj: ReportedObject) -> bool:
        return (
            obj.change_confidence >= self.min_change_confidence
            and obj.num_frames_observed >= self.min_frames
        )

    def changes(
        self,
        robot_name: str,
        removed: List[ReportedObject],
        added: List[ReportedObject],
        label_name: Optional[Callable[[int], str]],
    ) -> List[Change]:
        """The changes in this report not sent before.

        label_name maps AWCD's category id to a class name. While it is None (the robot's
        labelspace has not arrived) added objects without a name are held back -- not marked
        sent -- so that calling again with the same report once it is known sends them with
        their class.
        """
        out = []
        for obj in removed:
            symbol = removed_symbol(obj.obj_id)
            if symbol in self._removed_sent or not self._passes(obj):
                continue
            self._removed_sent.add(symbol)
            out.append(
                Change(REMOVED, symbol, "", (0.0, 0.0, 0.0), obj.change_confidence)
            )
        for obj in added:
            key = (robot_name, int(obj.obj_id))
            if key in self._added_sent or not self._passes(obj):
                continue
            if obj.name:
                label = obj.name
            elif label_name is None:
                continue
            else:
                label = (
                    label_name(obj.semantic_label) if obj.semantic_label >= 0 else ""
                )
            self._added_sent.add(key)
            out.append(Change(ADDED, "", label, obj.position, obj.change_confidence))
        return out

    def snapshot(
        self,
        robot_name: str,
        removed: List[ReportedObject],
        added: List[ReportedObject],
        label_name: Optional[Callable[[int], str]],
    ) -> List[Change]:
        """Every change in this report, keyed by AWCD's id, for the writer to mirror.

        Added objects without a class yet (label_name None, no name in the report) are left
        out, as in changes(); the next call with the labelspace known includes them.
        """
        out = []
        for obj in removed:
            if self._passes(obj):
                symbol = removed_symbol(obj.obj_id)
                out.append(
                    Change(REMOVED, symbol, "", (0.0, 0.0, 0.0), obj.change_confidence)
                )
        for obj in added:
            if not self._passes(obj):
                continue
            if obj.name:
                label = obj.name
            elif label_name is None:
                continue
            else:
                label = (
                    label_name(obj.semantic_label) if obj.semantic_label >= 0 else ""
                )
            self._added_sent.add((robot_name, int(obj.obj_id)))
            out.append(
                Change(
                    ADDED,
                    added_key(robot_name, obj.obj_id),
                    label,
                    obj.position,
                    obj.change_confidence,
                )
            )
        return out

    def held_back(self, robot_name: str, added: List[ReportedObject]) -> int:
        """How many added objects in this report have not been sent."""
        return sum((robot_name, int(o.obj_id)) not in self._added_sent for o in added)


def added_key(robot_name: str, obj_id: int) -> str:
    """AWCD's id for an added object, as the writer keys it (sent in the symbol field)."""
    return f"awcd:{robot_name}:{int(obj_id)}"


def snapshot_signature(changes: List[Change], grid_m: float = 0.1) -> tuple:
    """What a full change set says, to send it only when that changes (positions to grid_m)."""
    return tuple(
        sorted(
            (
                c.kind,
                c.symbol,
                c.semantic_label,
                tuple(round(v / grid_m) for v in c.position),
            )
            for c in changes
        )
    )


def labelspace_lookup(names: List[str]) -> Callable[[int], str]:
    """semantic_inference_msgs/Labelspace.names -> category id to name ("" if unnamed)."""
    table: Dict[int, str] = dict(enumerate(names))
    return lambda label: table.get(int(label), "")
