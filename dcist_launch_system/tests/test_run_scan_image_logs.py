"""run_scan on runs whose image store is built by hydra_image_store's indexer: counts
from the indexed store, raw log counts (zips / track dirs) before any indexing, and the
raw logs kept out of the manifest digest (a packed copy does not carry them)."""

import json
import os
import sqlite3

from dcist_launch_system.fleet import run_scan


def _raw_run(tmp_path):
    run = tmp_path / "run"
    (run / "hydra" / "backend").mkdir(parents=True)
    (run / "hydra" / "backend" / "dsg.json").write_text("{}")
    kf = run / "keyframes"
    kf.mkdir()
    for t in (1, 2, 3):
        (kf / f"{t}000000000.zip").write_bytes(b"PK")
    (kf / "4000000000.zip.tmp.ab12").write_bytes(b"PK")  # being written: not counted
    for tid in (7, 12):
        (run / "tracks" / str(tid)).mkdir(parents=True)
        (run / "tracks" / str(tid) / "track.json").write_text("{}")
    (run / "tracks" / "99").mkdir()  # no track.json yet
    (run / "events").mkdir()
    (run / "events" / "frontend.jsonl").write_text("{}\n")
    return run


def _store(run, tail=None):
    d = run / "image_store"
    d.mkdir()
    c = sqlite3.connect(str(d / "store.sqlite"))
    c.executescript(
        "CREATE TABLE meta (key TEXT PRIMARY KEY, value TEXT NOT NULL);"
        "CREATE TABLE sensors (sensor_id INTEGER PRIMARY KEY);"
        "CREATE TABLE frames (frame_id INTEGER PRIMARY KEY, sensor_id INTEGER, kind INTEGER,"
        " anchor_node_id INTEGER, rgb_path TEXT, depth_path TEXT);"
        "CREATE TABLE observations (obs_id INTEGER PRIMARY KEY);"
        "INSERT INTO meta VALUES ('schema_version', '1');"
        "INSERT INTO sensors VALUES (1);"
        "INSERT INTO frames VALUES (1, 1, 0, 5, NULL, NULL), (2, 1, 2, 5, NULL, NULL);"
    )
    if tail is not None:
        c.execute("INSERT INTO meta VALUES ('indexer.tail', ?)", (json.dumps(tail),))
    c.commit()
    c.close()


def test_raw_logs_only(tmp_path):
    run = _raw_run(tmp_path)
    e = run_scan.scan_run(str(run), measure=False)
    st = e["image_store"]
    assert e["has_image_store"] and st["indexed"] is False
    assert st["raw"] == {"keyframe_archives": 3, "tracks": 2, "events": True}
    assert e["store_fingerprint"] == "raw:3:2"


def test_indexed_store_counts_and_tail(tmp_path):
    run = _raw_run(tmp_path)
    _store(
        run,
        tail={
            "live": True,
            "final": False,
            "heartbeat_wall": 1.0,
            "session": 42,
            "polls": 3,
        },
    )
    st = run_scan.scan_run(str(run), measure=False)["image_store"]
    assert st["indexed"] is True and st["frames"] == {"keyframe": 1, "object": 1}
    assert st["raw"]["keyframe_archives"] == 3
    assert st["tail"] == {
        "live": True,
        "final": False,
        "heartbeat_wall": 1.0,
        "session": 42,
    }
    assert st["check"]["ok"], st["check"]


def test_store_with_sub_keyframes_needs_convert(tmp_path):
    run = _raw_run(tmp_path)
    _store(run)
    c = sqlite3.connect(str(run / "image_store" / "store.sqlite"))
    c.execute("UPDATE frames SET kind = 1 WHERE frame_id = 2")
    c.commit()
    c.close()
    st = run_scan.scan_run(str(run), measure=False)["image_store"]
    assert st["needs_convert"] is True and st["frames"] == {
        "keyframe": 1,
        "subkeyframe": 1,
    }
    assert not st["check"]["ok"]
    assert "hydra-image-store-adt4 convert" in " ".join(st["check"]["problems"])


def test_no_logs_no_store(tmp_path):
    run = tmp_path / "run"
    (run / "hydra").mkdir(parents=True)
    e = run_scan.scan_run(str(run), measure=False)
    assert e["image_store"] is None and not e["has_image_store"]


def test_manifest_skips_raw_logs(tmp_path):
    run = _raw_run(tmp_path)
    m1 = run_scan.manifest(str(run))
    (run / "keyframes" / "5000000000.zip").write_bytes(b"PK")
    (run / "tracks" / "8").mkdir()
    assert run_scan.manifest(str(run))["digest"] == m1["digest"]
    (run / "events" / "backend.jsonl").write_text("{}\n")  # events go with the copy
    assert run_scan.manifest(str(run))["digest"] != m1["digest"]
    # a "keyframes" dir deeper in the run is not a raw log dir
    m2 = run_scan.manifest(str(run))
    os.makedirs(run / "roman" / "keyframes")
    (run / "roman" / "keyframes" / "x").write_text("1")
    assert run_scan.manifest(str(run))["digest"] != m2["digest"]
