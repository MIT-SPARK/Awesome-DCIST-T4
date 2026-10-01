"""Unit tests for AwcdChangeStore (no ROS dependencies)."""

import numpy as np

from dcist_launch_system.awcd_change_store import (
    ADDED,
    REMOVED,
    AwcdChangeStore,
    ChangeRecord,
)


def _record(robot_name, kind, obj_id, stamp_ns=100, last_seen_ns=1000):
    return ChangeRecord(
        robot_name=robot_name,
        kind=kind,
        obj_id=obj_id,
        stamp_ns=stamp_ns,
        center=np.zeros(3),
        dimensions=np.ones(3),
        orientation=np.array([0.0, 0.0, 0.0, 1.0]),
        semantic_label=1,
        confidence=0.9,
        change_confidence=0.75,
        num_frames_observed=5,
        last_seen_ns=last_seen_ns,
    )


class TestAwcdChangeStore:
    def test_update_stores_records(self):
        store = AwcdChangeStore()
        store.update("hamilton", ADDED, [_record("hamilton", ADDED, 1)])

        records = store.records("hamilton", ADDED)
        assert len(records) == 1
        assert records[0].obj_id == 1

    def test_update_replaces_slice_wholesale(self):
        store = AwcdChangeStore()
        store.update("hamilton", ADDED, [_record("hamilton", ADDED, 1, stamp_ns=100)])
        # Republish with a different stamp -- the message is authoritative, so the new stamp
        # must win (no latching/preservation across messages).
        store.update("hamilton", ADDED, [_record("hamilton", ADDED, 1, stamp_ns=999)])

        records = store.records("hamilton", ADDED)
        assert len(records) == 1
        assert records[0].stamp_ns == 999

    def test_empty_update_clears_slice(self):
        store = AwcdChangeStore()
        store.update("hamilton", ADDED, [_record("hamilton", ADDED, 1)])
        store.update("hamilton", ADDED, [])

        assert store.records("hamilton", ADDED) == []

    def test_update_drops_ids_no_longer_reported(self):
        store = AwcdChangeStore()
        store.update(
            "hamilton",
            ADDED,
            [_record("hamilton", ADDED, 1), _record("hamilton", ADDED, 2)],
        )
        store.update("hamilton", ADDED, [_record("hamilton", ADDED, 1)])

        ids = {r.obj_id for r in store.records("hamilton", ADDED)}
        assert ids == {1}

    def test_robots_with_identical_track_ids_stay_separate(self):
        store = AwcdChangeStore()
        store.update("hamilton", ADDED, [_record("hamilton", ADDED, 5)])
        store.update("lewis", ADDED, [_record("lewis", ADDED, 5)])

        assert len(store.records(kind=ADDED)) == 2
        assert {r.robot_name for r in store.records(kind=ADDED)} == {
            "hamilton",
            "lewis",
        }
        assert store.robots() == ["hamilton", "lewis"]

    def test_added_and_removed_do_not_collide_on_same_id(self):
        store = AwcdChangeStore()
        store.update("hamilton", ADDED, [_record("hamilton", ADDED, 1)])
        store.update("hamilton", REMOVED, [_record("hamilton", REMOVED, 1)])

        assert len(store.records("hamilton", ADDED)) == 1
        assert len(store.records("hamilton", REMOVED)) == 1

    def test_drop_robot_only_removes_that_robot(self):
        store = AwcdChangeStore()
        store.update("hamilton", ADDED, [_record("hamilton", ADDED, 1)])
        store.update("lewis", ADDED, [_record("lewis", ADDED, 1)])

        store.drop_robot("hamilton")

        assert store.records("hamilton") == []
        assert len(store.records("lewis")) == 1
        assert store.robots() == ["lewis"]
