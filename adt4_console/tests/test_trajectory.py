from adt4_console.graph_service import trajectory_payload


def test_trajectory_split_per_robot_and_at_gaps():
    rows = [
        [2.0, 1.0, 0.0, 0.0, "a"],
        [1.0, 0.0, 0.0, 0.0, "a"],  # unordered
        [3.0, 41.0, 0.0, 0.0, "a"],  # 40 m without keyframes: a gap
        [1.0, 0.0, 5.0, 0.0, "b"],
        [2.0, 1.0, 5.0, 0.0, "b"],
    ]
    out = trajectory_payload(rows)
    assert [p[0] for p in out["points"]] == [1.0, 2.0, 3.0, 1.0, 2.0]
    assert all(len(p) == 4 for p in out["points"])
    assert out["breaks"] == [2, 3]
    assert [(r["key"], r["start"]) for r in out["robots"]] == [("a", 0), ("b", 3)]
    out = trajectory_payload(
        rows, {"b": {"session": "hamilton", "robot": "hamilton", "bag": "/bags/h"}}
    )
    assert out["robots"][1]["bag"] == "/bags/h" and "session" not in out["robots"][0]
    assert trajectory_payload([]) == {"points": [], "breaks": [], "robots": []}


def test_trajectory_symbols():
    rows = [[1.0, 0.0, 0.0, 0.0, "a", "a(0)"], [2.0, 1.0, 0.0, 0.0, "a", "a(1)"]]
    assert trajectory_payload(rows)["syms"] == ["a(0)", "a(1)"]
    assert "syms" not in trajectory_payload([r[:5] for r in rows])
