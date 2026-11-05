"""Unit tests for AwcdSceneChanges (no ROS dependencies)."""

import spark_dsg

from dcist_launch_system.awcd_scene_changes import (
    ADDED,
    REMOVED,
    AwcdSceneChanges,
    ReportedObject,
    added_key,
    labelspace_lookup,
    removed_symbol,
    snapshot_signature,
)

O16 = spark_dsg.NodeSymbol("O", 16).value
O17 = spark_dsg.NodeSymbol("O", 17).value
LABELS = labelspace_lookup(["", "unknown", "chair", "box"])


def _removed(node_id, cc=0.9, n=10):
    return ReportedObject(obj_id=node_id, change_confidence=cc, num_frames_observed=n)


def _added(obj_id, label=3, cc=0.9, n=10):
    return ReportedObject(
        obj_id=obj_id,
        change_confidence=cc,
        num_frames_observed=n,
        semantic_label=label,
        position=(1.0, 2.0, 0.5),
    )


class TestAwcdSceneChanges:
    def test_removed_id_becomes_the_map_symbol(self):
        assert removed_symbol(O16) == "O16"
        (c,) = AwcdSceneChanges().changes("hamilton", [_removed(O16)], [], LABELS)
        assert (c.kind, c.symbol) == (REMOVED, "O16")

    def test_a_removal_is_sent_once_across_reports_and_robots(self):
        f = AwcdSceneChanges()
        assert len(f.changes("hamilton", [_removed(O16)], [], LABELS)) == 1
        # The robot republishes its whole set when something else changes ...
        later = f.changes("hamilton", [_removed(O16), _removed(O17)], [], LABELS)
        assert [c.symbol for c in later] == ["O17"]
        # ... and another robot seeing the same object gone is the same removal.
        assert f.changes("apollo", [_removed(O16)], [], LABELS) == []

    def test_an_addition_is_sent_once_with_its_class_and_no_symbol(self):
        f = AwcdSceneChanges()
        (c,) = f.changes("hamilton", [], [_added(7)], LABELS)
        assert (c.kind, c.symbol, c.semantic_label) == (ADDED, "", "box")
        assert c.position == (1.0, 2.0, 0.5)
        # Republished alongside a new one: only the new one is sent.
        assert len(f.changes("hamilton", [], [_added(7), _added(8)], LABELS)) == 1
        assert f.changes("hamilton", [], [_added(7), _added(8)], LABELS) == []

    def test_added_ids_are_per_robot(self):
        f = AwcdSceneChanges()
        f.changes("hamilton", [], [_added(7)], LABELS)
        assert len(f.changes("apollo", [], [_added(7)], LABELS)) == 1

    def test_added_objects_wait_for_the_labelspace(self):
        f = AwcdSceneChanges()
        report = ([_removed(O16)], [_added(7)])
        held = f.changes("hamilton", *report, None)
        assert [c.kind for c in held] == [REMOVED]  # removals need no class
        assert f.held_back("hamilton", report[1]) == 1
        (c,) = f.changes("hamilton", *report, LABELS)  # the same report, re-offered
        assert (c.kind, c.semantic_label) == (ADDED, "box")
        assert f.held_back("hamilton", report[1]) == 0

    def test_unlabelled_and_unnamed_classes_are_empty(self):
        f = AwcdSceneChanges()
        changes = f.changes(
            "hamilton", [], [_added(7, label=-1), _added(8, label=99)], LABELS
        )
        assert [c.semantic_label for c in changes] == ["", ""]

    def test_thresholds(self):
        f = AwcdSceneChanges(min_change_confidence=0.5, min_frames=5)
        changes = f.changes(
            "hamilton",
            [_removed(O16, cc=0.4), _removed(O17, n=4)],
            [_added(7, cc=0.6, n=5)],
            LABELS,
        )
        assert [(c.kind, c.symbol) for c in changes] == [(ADDED, "")]


class TestReconciledReports:
    def test_an_added_object_with_a_name_needs_no_labelspace(self):
        obj = _added(7, label=-1)
        obj.name = "box"
        (c,) = AwcdSceneChanges().changes("reconciled", [], [obj], None)
        assert (c.kind, c.semantic_label) == (ADDED, "box")

    def test_a_name_wins_over_the_category_id(self):
        obj = _added(7, label=2)
        obj.name = "box"
        (c,) = AwcdSceneChanges().changes("reconciled", [], [obj], LABELS)
        assert c.semantic_label == "box"


class TestFullChangeSet:
    def test_every_change_is_sent_each_time_with_awcds_id(self):
        f = AwcdSceneChanges()
        first = f.snapshot("reconciled", [_removed(O16)], [_added(7)], LABELS)
        again = f.snapshot("reconciled", [_removed(O16)], [_added(7)], LABELS)
        assert [(c.kind, c.symbol) for c in first] == [
            (REMOVED, "O16"),
            (ADDED, added_key("reconciled", 7)),
        ]
        assert snapshot_signature(first) == snapshot_signature(again)

    def test_an_empty_report_is_an_empty_set(self):
        f = AwcdSceneChanges()
        f.snapshot("reconciled", [_removed(O16)], [], LABELS)
        assert f.snapshot("reconciled", [], [], LABELS) == []

    def test_added_objects_wait_for_their_class(self):
        f = AwcdSceneChanges()
        assert f.snapshot("hamilton", [], [_added(7)], None) == []
        assert f.held_back("hamilton", [_added(7)]) == 1
        (c,) = f.snapshot("hamilton", [], [_added(7)], LABELS)
        assert c.semantic_label == "box" and f.held_back("hamilton", [_added(7)]) == 0

    def test_small_position_updates_do_not_change_the_signature(self):
        f = AwcdSceneChanges()
        a = f.snapshot("reconciled", [], [_added(7)], LABELS)
        moved = _added(7)
        moved.position = (1.02, 2.0, 0.5)
        b = f.snapshot("reconciled", [], [moved], LABELS)
        far = _added(7)
        far.position = (1.5, 2.0, 0.5)
        c = f.snapshot("reconciled", [], [far], LABELS)
        assert snapshot_signature(a) == snapshot_signature(b) != snapshot_signature(c)

    def test_thresholds_still_apply(self):
        f = AwcdSceneChanges(min_change_confidence=0.5)
        assert f.snapshot("reconciled", [_removed(O16, cc=0.2)], [], LABELS) == []
