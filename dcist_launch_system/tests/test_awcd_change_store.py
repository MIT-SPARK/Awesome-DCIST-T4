"""Unit tests for AwcdChangeStore (no ROS dependencies)."""

import numpy as np

from dcist_launch_system.awcd_change_store import (
    ADDED,
    RECONCILED,
    REMOVED,
    AwcdChangeStore,
    ChangeRecord,
    LabelDropFilter,
    PriorObject,
    PriorObjectOverlapFilter,
    ReconcileConfig,
    _namespace_crc,
    reconciled_added_id,
)


def _record(
    namespace,
    kind,
    obj_id,
    stamp_ns=100,
    last_seen_ns=1000,
    center=(0.0, 0.0, 0.0),
    semantic_label=1,
    num_frames_observed=5,
):
    return ChangeRecord(
        namespace=namespace,
        kind=kind,
        obj_id=obj_id,
        stamp_ns=stamp_ns,
        center=np.array(center, dtype=float),
        dimensions=np.ones(3),
        orientation=np.array([0.0, 0.0, 0.0, 1.0]),
        semantic_label=semantic_label,
        confidence=0.9,
        change_confidence=0.75,
        num_frames_observed=num_frames_observed,
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
        assert {r.namespace for r in store.records(kind=ADDED)} == {
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


def _reconciled(store, kind=None):
    store.reconcile()
    return store.reconciled(kind)


def _member_keys(record):
    return {(m.namespace, m.obj_id) for m in record.members}


class TestReconciled:
    def test_same_removed_node_id_merges_across_robots(self):
        store = AwcdChangeStore()
        store.update(
            "hamilton", REMOVED, [_record("hamilton", REMOVED, 7, stamp_ns=300)]
        )
        store.update("lewis", REMOVED, [_record("lewis", REMOVED, 7, stamp_ns=200)])

        (record,) = _reconciled(store)
        assert record.namespace == RECONCILED
        assert record.kind == REMOVED
        assert record.obj_id == 7
        assert record.stamp_ns == 200
        assert _member_keys(record) == {("hamilton", 7), ("lewis", 7)}

    def test_overlapping_added_boxes_merge_across_robots(self):
        store = AwcdChangeStore()
        store.update("hamilton", ADDED, [_record("hamilton", ADDED, 1, stamp_ns=500)])
        store.update(
            "lewis",
            ADDED,
            [
                _record(
                    "lewis",
                    ADDED,
                    9,
                    stamp_ns=400,
                    center=(0.3, 0.0, 0.0),
                    num_frames_observed=8,
                )
            ],
        )

        (record,) = _reconciled(store)
        assert _member_keys(record) == {("hamilton", 1), ("lewis", 9)}
        # Id from the earliest member, bbox from the best-observed one.
        assert record.obj_id == reconciled_added_id("lewis", 9)
        assert np.allclose(record.center, [0.3, 0.0, 0.0])
        assert record.num_frames_observed == 13

    def test_close_but_disjoint_boxes_do_not_merge(self):
        store = AwcdChangeStore()
        store.update("hamilton", ADDED, [_record("hamilton", ADDED, 1)])
        store.update(
            "lewis", ADDED, [_record("lewis", ADDED, 1, center=(1.1, 0.0, 0.0))]
        )

        assert len(_reconciled(store)) == 2

    def test_same_robot_records_merge(self):
        # A revisit: the robot reports the same object again under a new track id.
        store = AwcdChangeStore()
        store.update(
            "hamilton",
            ADDED,
            [
                _record("hamilton", ADDED, 1, stamp_ns=100),
                _record("hamilton", ADDED, 2, stamp_ns=900, center=(0.1, 0.0, 0.0)),
            ],
        )

        (record,) = _reconciled(store)
        assert _member_keys(record) == {("hamilton", 1), ("hamilton", 2)}
        assert record.obj_id == reconciled_added_id("hamilton", 1)

    def test_label_mismatch_does_not_merge(self):
        store = AwcdChangeStore()
        store.update(
            "hamilton", ADDED, [_record("hamilton", ADDED, 1, semantic_label=1)]
        )
        store.update("lewis", ADDED, [_record("lewis", ADDED, 1, semantic_label=2)])

        assert len(_reconciled(store)) == 2

        store.config = ReconcileConfig(require_same_label=False)
        store.update("lewis", ADDED, [_record("lewis", ADDED, 1, semantic_label=2)])
        assert len(_reconciled(store)) == 1

    def test_unknown_label_cannot_bridge_two_labels(self):
        store = AwcdChangeStore()
        store.update("a", ADDED, [_record("a", ADDED, 1, semantic_label=1)])
        store.update("b", ADDED, [_record("b", ADDED, 1, semantic_label=-1)])
        store.update("c", ADDED, [_record("c", ADDED, 1, semantic_label=2)])

        groups = sorted(len(r.members) for r in _reconciled(store))
        assert groups == [1, 2]

    def test_group_can_hold_several_records_of_one_robot(self):
        # hamilton:1 and hamilton:2 both overlap lewis:1, so all three are one object.
        store = AwcdChangeStore()
        store.update(
            "hamilton",
            ADDED,
            [
                _record("hamilton", ADDED, 1),
                _record("hamilton", ADDED, 2, center=(0.6, 0.0, 0.0)),
            ],
        )
        store.update(
            "lewis", ADDED, [_record("lewis", ADDED, 1, center=(0.1, 0.0, 0.0))]
        )

        (record,) = _reconciled(store)
        assert _member_keys(record) == {("hamilton", 1), ("hamilton", 2), ("lewis", 1)}

    def test_reconciled_added_ids_are_unique(self):
        store = AwcdChangeStore()
        store.update(
            "hamilton",
            ADDED,
            [
                _record("hamilton", ADDED, 1),
                _record("hamilton", ADDED, 2, center=(0.1, 0.0, 0.0)),
                _record("hamilton", ADDED, 3, center=(5.0, 0.0, 0.0)),
            ],
        )
        store.update(
            "lewis",
            ADDED,
            [
                _record("lewis", ADDED, 1, center=(5.1, 0.0, 0.0)),
                _record("lewis", ADDED, 2, center=(9.0, 0.0, 0.0)),
            ],
        )

        ids = [r.obj_id for r in _reconciled(store, ADDED)]
        assert len(ids) == 3
        assert len(set(ids)) == len(ids)

    def test_rotated_box_uses_enclosing_aabb(self):
        store = AwcdChangeStore()
        long_box = _record("hamilton", ADDED, 1)
        long_box.dimensions = np.array([3.0, 1.0, 1.0])
        store.update("hamilton", ADDED, [long_box])
        store.update(
            "lewis", ADDED, [_record("lewis", ADDED, 1, center=(0.0, 1.2, 0.0))]
        )
        assert len(_reconciled(store)) == 2

        # 90 degrees about z: the box now extends along y and overlaps the other one.
        long_box.orientation = np.array([0.0, 0.0, np.sqrt(0.5), np.sqrt(0.5)])
        store.update("hamilton", ADDED, [long_box])
        assert len(_reconciled(store)) == 1

    def test_added_id_is_deterministic_and_robot_specific(self):
        assert reconciled_added_id("hamilton", 5) == reconciled_added_id("hamilton", 5)
        assert reconciled_added_id("hamilton", 5) != reconciled_added_id("lewis", 5)
        assert 0 <= reconciled_added_id("hamilton", 5) < 2**63

    def test_reconciled_is_kept_separate_from_per_robot_records(self):
        store = AwcdChangeStore()
        store.update("hamilton", REMOVED, [_record("hamilton", REMOVED, 7)])
        store.update("lewis", REMOVED, [_record("lewis", REMOVED, 7)])

        assert store.robots() == ["hamilton", "lewis"]
        assert len(store.records(kind=REMOVED)) == 2
        assert len(_reconciled(store, REMOVED)) == 1
        assert _reconciled(store, ADDED) == []

    def test_reconciled_follows_updates(self):
        store = AwcdChangeStore()
        store.update("hamilton", REMOVED, [_record("hamilton", REMOVED, 7)])
        assert len(_reconciled(store)) == 1

        store.update("hamilton", REMOVED, [])
        assert _reconciled(store) == []
        store.update("lewis", REMOVED, [_record("lewis", REMOVED, 8)])
        store.drop_robot("lewis")
        assert _reconciled(store) == []

    def test_reconciled_is_unchanged_until_reconcile(self):
        store = AwcdChangeStore()
        store.update("hamilton", REMOVED, [_record("hamilton", REMOVED, 7)])
        assert store.reconciled() == []

        store.reconcile()
        assert len(store.reconciled()) == 1

    def test_colliding_robot_names_are_rejected(self):
        store = AwcdChangeStore()
        store.update("hamilton", ADDED, [])
        # Force a crc collision by pre-registering the crc under another name.
        store._namespace_crcs[_namespace_crc("lewis")] = "not_lewis"
        try:
            store.update("lewis", ADDED, [])
        except ValueError:
            return
        raise AssertionError("expected ValueError")


def _prior(
    obj_id, center=(0.0, 0.0, 0.0), semantic_label=1, dimensions=(1.0, 1.0, 1.0)
):
    return PriorObject(
        obj_id=obj_id,
        center=np.array(center, dtype=float),
        dimensions=np.array(dimensions, dtype=float),
        orientation=np.array([0.0, 0.0, 0.0, 1.0]),
        semantic_label=semantic_label,
    )


class TestFilters:
    def _store(self, *filters):
        store = AwcdChangeStore(filters=list(filters))
        store.set_prior_objects([_prior(7, semantic_label=1)])
        return store

    def test_added_inside_same_label_prior_object_is_dropped(self):
        store = self._store(PriorObjectOverlapFilter())
        store.update(
            "hamilton", ADDED, [_record("hamilton", ADDED, 1, center=(0.2, 0.0, 0.0))]
        )

        assert _reconciled(store) == []
        # Per-robot records are not filtered.
        assert len(store.records(kind=ADDED)) == 1

    def test_added_inside_other_label_prior_object_is_kept(self):
        store = self._store(PriorObjectOverlapFilter())
        store.update(
            "hamilton",
            ADDED,
            [_record("hamilton", ADDED, 1, center=(0.2, 0.0, 0.0), semantic_label=2)],
        )
        assert len(_reconciled(store)) == 1

        store.filters = [
            PriorObjectOverlapFilter(
                PriorObjectOverlapFilter.Config(require_same_label=False)
            )
        ]
        assert _reconciled(store) == []

    def test_added_barely_overlapping_prior_object_is_kept(self):
        store = self._store(PriorObjectOverlapFilter())
        store.update(
            "hamilton", ADDED, [_record("hamilton", ADDED, 1, center=(0.8, 0.0, 0.0))]
        )
        assert len(_reconciled(store)) == 1

    def test_removed_prior_object_does_not_drop_added(self):
        # The prior object was taken away and something of the same class now stands there.
        store = self._store(
            PriorObjectOverlapFilter(),
            LabelDropFilter(LabelDropFilter.Config(removed_labels=[1])),
        )
        store.update(
            "hamilton", REMOVED, [_record("hamilton", REMOVED, 7, semantic_label=1)]
        )
        store.update(
            "hamilton", ADDED, [_record("hamilton", ADDED, 1, center=(0.2, 0.0, 0.0))]
        )

        # The removed report itself is label-dropped, but still counts as removed for the overlap.
        (record,) = _reconciled(store)
        assert record.kind == ADDED

    def test_label_drop_is_per_kind(self):
        store = AwcdChangeStore(
            filters=[
                LabelDropFilter(
                    LabelDropFilter.Config(added_labels=[3], removed_labels=[4])
                )
            ]
        )
        store.update(
            "hamilton",
            ADDED,
            [
                _record("hamilton", ADDED, 1, semantic_label=3),
                _record("hamilton", ADDED, 2, center=(5.0, 0.0, 0.0), semantic_label=4),
            ],
        )
        store.update(
            "hamilton",
            REMOVED,
            [
                _record("hamilton", REMOVED, 8, semantic_label=3),
                _record("hamilton", REMOVED, 9, semantic_label=4),
            ],
        )

        kept = {(r.kind, r.obj_id) for r in _reconciled(store)}
        assert kept == {(ADDED, reconciled_added_id("hamilton", 2)), (REMOVED, 8)}
