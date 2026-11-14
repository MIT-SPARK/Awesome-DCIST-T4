"""Run identity: run_info.json, base-side naming, copy pairing, launch prior by run_id."""

import json
import os
import pathlib
import subprocess

import pytest
from test_launch_workflow import OUT, RUNTIME, TOPO, FakeSsh, Recorder, make_deps

from dcist_launch_system.fleet import run_identity as ri
from dcist_launch_system.fleet import run_scan
from dcist_launch_system.fleet.launch_workflow import LaunchPlan, run_launch


def make_run(root: pathlib.Path, name: str) -> pathlib.Path:
    run = root / name
    (run / "hydra" / "backend").mkdir(parents=True)
    (run / "roman").mkdir()
    (run / "hydra" / "backend" / "dsg.json").write_text("{}")
    (run / "roman" / "1.roman_map.pkl").write_bytes(b"r" * 100)
    return run


# --- run_info.json -----------------------------------------------------------


def test_write_run_info_creates_once_and_never_overwrites(tmp_path):
    run = tmp_path / "map_20261019_2308"
    info = ri.write_run_info(
        run, robot="hamilton", platform="smaug", session="spot-default"
    )
    assert (run / "run_info.json").is_file()
    assert info["experiment"] == "map_20261019_2308" and info["robot"] == "hamilton"
    assert info["platform"] == "smaug" and info["session"] == "spot-default"
    assert info["started_at"].endswith("+00:00") and info["host"]
    assert len(info["run_id"]) == 36
    # a restart into the same directory keeps the id
    again = ri.write_run_info(run, robot="other", platform="x", session="y")
    assert again == info
    assert json.loads((run / "run_info.json").read_text())["run_id"] == info["run_id"]


def test_write_run_info_replaces_garbage_and_records_commit(tmp_path):
    run = tmp_path / "r"
    run.mkdir()
    (run / "run_info.json").write_text("not json")
    info = ri.write_run_info(run, robot="a", commit="0" * 40)
    assert info["git_commit"] == "0" * 40
    assert (run / "run_info.json.invalid").read_text() == "not json"
    assert ri.read_run_info(run)["run_id"] == info["run_id"]


def test_git_commit_of_this_checkout():
    sha = ri.git_commit(pathlib.Path(__file__).parent)
    assert sha is None or len(sha) == 40


def test_run_adt4_writes_run_info(tmp_path):
    import importlib.machinery
    import importlib.util

    path = pathlib.Path(__file__).resolve().parents[1] / "bin" / "run-adt4"
    loader = importlib.machinery.SourceFileLoader("run_adt4_ri", str(path))
    spec = importlib.util.spec_from_loader("run_adt4_ri", loader)
    mod = importlib.util.module_from_spec(spec)
    loader.exec_module(mod)
    if not hasattr(mod, "_write_run_info"):
        # develop-nov11's run-adt4 does not write run_info.json; the scan falls back
        # to the directory identity (test_scan_identity_run_info_origin_and_fallback)
        pytest.skip("run-adt4 does not write run_info.json on this branch")
    out = tmp_path / "exp"
    mod._write_run_info(out, "hamilton", "smaug", pathlib.Path("/x/spot-default.yaml"))
    info = json.loads((out / "run_info.json").read_text())
    assert info["robot"] == "hamilton" and info["session"] == "spot-default"
    first = info["run_id"]
    mod._write_run_info(out, "hamilton", "smaug", pathlib.Path("/x/spot-default.yaml"))
    assert json.loads((out / "run_info.json").read_text())["run_id"] == first


# --- identity in the scan ----------------------------------------------------


def test_scan_identity_run_info_origin_and_fallback(tmp_path):
    a = make_run(tmp_path, "with_info")
    info = ri.write_run_info(a, robot="hamilton")
    b = make_run(tmp_path, "legacy")
    c = make_run(tmp_path, "copy__hamilton")
    (c / ri.ORIGIN).write_text(
        json.dumps({"run_id": "abc", "source_machine": "hamilton"})
    )

    ea = run_scan.scan_run(str(a), measure=False)
    assert (
        ea["identity"]["run_id"] == info["run_id"]
        and ea["identity"]["source"] == "run_info"
    )
    eb = run_scan.scan_run(str(b), measure=False)
    assert eb["identity"]["source"] == "fallback" and eb["identity"][
        "run_id"
    ].startswith("legacy-")
    # stable across scans, nothing written
    assert (
        run_scan.scan_run(str(b), measure=False)["identity"]["run_id"]
        == eb["identity"]["run_id"]
    )
    assert sorted(os.listdir(b)) == ["hydra", "roman"]
    ec = run_scan.scan_run(str(c), measure=False)
    assert ec["identity"] == {"run_id": "abc", "source": "origin"}
    assert ec["origin"]["source_machine"] == "hamilton"
    light = run_scan.scan_root(str(tmp_path), light=True)
    assert {r["name"]: r["identity"]["run_id"] for r in light}["with_info"] == info[
        "run_id"
    ]
    assert "manifest" not in light[0]


def test_manifest_survives_rsync_and_ignores_store_and_origin(tmp_path):
    src = make_run(tmp_path / "robot", "map_x")
    (src / "image_store").mkdir()
    (src / "image_store" / "store.sqlite").write_bytes(b"db")
    (src / "recorded_data").mkdir()
    (src / "recorded_data" / "bag.mcap").write_bytes(b"b" * 10)
    dst = tmp_path / "base" / "map_x__hamilton"
    dst.parent.mkdir()
    subprocess.run(
        [
            "rsync",
            "-a",
            "--exclude",
            "recorded_data/",
            "--exclude",
            "/image_store/",
            f"{src}/",
            f"{dst}/",
        ],
        check=True,
    )
    (dst / ri.ORIGIN).write_text("{}")
    m_src, m_dst = run_scan.manifest(str(src)), run_scan.manifest(str(dst))
    assert m_src["digest"] == m_dst["digest"] and m_src["files"] == 2
    (src / "roman" / "2.roman_map.pkl").write_bytes(b"new")
    assert run_scan.manifest(str(src))["digest"] != m_dst["digest"]


# --- naming ------------------------------------------------------------------


def test_copy_name_appends_robot_unless_named():
    assert (
        ri.copy_name("map_20261019_2308", "hamilton") == "map_20261019_2308__hamilton"
    )
    assert ri.copy_name("hamilton_floor6", "hamilton") == "hamilton_floor6"
    assert ri.copy_name("map_0730-Hamilton", "hamilton") == "map_0730-Hamilton"
    assert ri.copy_name("hamiltonian_test", "hamilton") == "hamiltonian_test__hamilton"
    assert ri.copy_name("x", None) == "x"


def test_choose_dest_name_collision_suffix():
    assert ri.choose_dest_name("e__h", {}, "id1") == ("e__h", "new")
    assert ri.choose_dest_name("e__h", {"e__h": {"id1"}}, "id1") == ("e__h", "update")
    taken = {"e__h": {"other"}}
    assert ri.choose_dest_name("e__h", taken, "id1") == ("e__h__2", "collision")
    taken["e__h__2"] = {"id1"}
    assert ri.choose_dest_name("e__h", taken, "id1") == ("e__h__2", "collision_update")
    taken["e__h__2"] = set()  # unknown identity: never reused
    assert ri.choose_dest_name("e__h", taken, "id1") == ("e__h__3", "collision")


# --- pairing -----------------------------------------------------------------


def _entry(
    name,
    rid,
    digest="d1",
    store="5:1:keyframe=5",
    path=None,
    origin=None,
    source="run_info",
):
    return {
        "name": name,
        "path": path or f"~/adt4_output/{name}",
        "abs_path": path or f"/home/swarm/adt4_output/{name}",
        "identity": {"run_id": rid, "source": source},
        "origin": origin,
        "manifest": {"digest": digest},
        "store_fingerprint": store,
        "has_image_store": store is not None,
    }


def _origin(
    rid,
    digest="d1",
    store="5:1:keyframe=5",
    images="keyframes",
    path="/home/swarm/adt4_output/map_x",
):
    return {
        "run_id": rid,
        "source_machine": "hamilton",
        "source_path": path,
        "manifest_digest": digest,
        "store_fingerprint": store,
        "images": images,
        "compression": "fast",
        "transferred_at": "2026-10-20T04:00:00+00:00",
    }


def test_pair_states_in_sync_outdated_not_copied():
    hamilton = [
        _entry("map_x", "A"),
        _entry("map_y", "B", digest="d2-new"),
        _entry("map_z", "C"),
        _entry("map_s", "D", store="9:1:keyframe=9"),
    ]
    willow = [
        _entry("map_x", "W"),  # the base's own run of the same experiment
        _entry("map_x__hamilton", "x", origin=_origin("A"), source="origin"),
        _entry(
            "map_y__hamilton", "x", origin=_origin("B", digest="d2"), source="origin"
        ),
        _entry("map_s__hamilton", "x", origin=_origin("D"), source="origin"),
        _entry(
            "from_offline__sequoia",
            "Q",
            origin=dict(_origin("Q"), source_machine="sequoia"),
            source="origin",
        ),
    ]
    machines = {
        "hamilton": hamilton,
        "willow": willow,
        "gauss": {"error": "unreachable"},
    }
    ri.pair_copies(machines, "willow")
    st = {e["name"]: e["copy"] for e in hamilton}
    assert (
        st["map_x"]["state"] == "in_sync" and st["map_x"]["name"] == "map_x__hamilton"
    )
    assert st["map_x"]["matched_by"] == "run_id"
    assert st["map_y"]["state"] == "outdated"  # robot changed files since the copy
    assert st["map_s"]["state"] == "outdated"  # robot added frames to its image store
    assert st["map_z"] == {"state": "not_copied", "machine": "willow"}
    w = {e["name"]: e for e in willow}
    assert "copy_of" not in w["map_x"]  # same name, different run: not a copy
    assert w["map_x__hamilton"]["copy_of"]["path"] == "/home/swarm/adt4_output/map_x"
    assert w["map_x__hamilton"]["copy_of"]["state"] == "in_sync"
    assert w["from_offline__sequoia"]["copy_of"]["state"] == "unknown"


def test_pair_by_origin_path_and_legacy_contents():
    hamilton = [
        _entry("old_run", "legacy-1", source="fallback"),
        _entry("renamed", "legacy-2", source="fallback"),
    ]
    willow = [
        # copy made before run ids: same name, own fallback id, identical files
        _entry("old_run", "legacy-9", source="fallback"),
        # origin written with a different id (fallback changed) but same source path
        _entry(
            "renamed_copy",
            "zzz",
            origin=_origin("zzz", path="/home/swarm/adt4_output/renamed"),
            source="origin",
        ),
    ]
    ri.pair_copies({"hamilton": hamilton, "willow": willow}, "willow")
    assert (
        hamilton[0]["copy"]["state"] == "in_sync"
        and hamilton[0]["copy"]["matched_by"] == "contents"
    )
    assert hamilton[1]["copy"]["matched_by"] == "origin"
    assert willow[0]["copy_of"]["machine"] == "hamilton"


def test_store_needs_send():
    src = _entry("m", "A")
    assert ri.store_needs_send(src, None, "keyframes", "fast")
    assert not ri.store_needs_send(src, _origin("A"), "keyframes", "fast")
    assert ri.store_needs_send(src, _origin("A"), "all", "fast")
    assert ri.store_needs_send(
        dict(src, store_fingerprint="6:1:keyframe=6"), _origin("A"), "keyframes", "fast"
    )
    assert not ri.store_needs_send(src, None, "none", "none")


def test_make_origin_fields():
    src = dict(_entry("map_x", "A"), size_bytes=10, signature=5.0)
    o = ri.make_origin(src, "hamilton", "keyframes", "fast")
    assert o["run_id"] == "A" and o["source_path"] == "/home/swarm/adt4_output/map_x"
    assert (
        o["manifest_digest"] == "d1"
        and o["source_size"] == 10
        and o["robot"] == "hamilton"
    )
    o2 = ri.make_origin(src, "hamilton", "keyframes", "fast", previous=o)
    assert o2["first_transferred_at"] == o["transferred_at"]


def test_remote_write_command_roundtrip(tmp_path, monkeypatch):
    monkeypatch.setenv("HOME", str(tmp_path))
    cmd = ri.remote_write_command("~/o.json", {"run_id": "A"})
    subprocess.run(["sh", "-c", cmd], check=True)
    assert json.loads((tmp_path / "o.json").read_text()) == {"run_id": "A"}


# --- launch Phase 1 by run_id -------------------------------------------------


def _plan(**kw):
    base = dict(
        robots=["smaug", "topaz"],
        session="spot-relocalize",
        experiment="e",
        prior_path="~/adt4_output/map_x__hamilton",
        prior_source="willow",
        preflight=False,
    )
    base.update(kw)
    return LaunchPlan(**base)


def _deps(ssh, rsync, listings):
    deps = make_deps(ssh, rsync)

    def scan(machine, root, only=None, manifest=False):
        runs = listings.get(machine["name"])
        if runs is None:
            return None
        return [r for r in runs if only is None or r["name"] in only]

    deps.scan = scan
    return deps


def _ident(name, rid, subdirs=("hydra", "roman"), origin=None):
    return {
        "name": name,
        "subdirs": list(subdirs),
        "identity": {"run_id": rid},
        "origin": origin,
    }


def test_launch_prior_phase_skips_machines_with_same_run_id():
    listings = {
        "willow": [_ident("map_x__hamilton", "x", origin={"run_id": "RID"})],
        # smaug holds the original run under its own name: no transfer, use it
        "smaug": [_ident("map_x", "RID")],
        # topaz has a stale prior_ of the same name but another run: never merged into
        "topaz": [_ident("prior_map_x__hamilton", "OTHER")],
    }
    rsyncs = []
    ssh = FakeSsh()
    rec = Recorder()
    ok = run_launch(
        _plan(),
        topo=TOPO,
        active_network="silvus",
        runtime_config=RUNTIME,
        output_root=OUT,
        callbacks=rec,
        deps=_deps(
            ssh, lambda *a, **k: rsyncs.append(a) or (True, "direct", "ok"), listings
        ),
    )
    assert ok, rec.text()
    assert len(rsyncs) == 1 and rsyncs[0][5] == f"{OUT}/prior_map_x__hamilton__2"
    assert "smaug: prior run already present as map_x" in rec.text()
    assert "holds a different run" in rec.text()
    launch = {ip: c for ip, c in ssh.calls if "run-adt4" in c}
    assert (
        f"-p {OUT}/map_x " in launch["10.0.0.1"].replace("'", "")
        or "map_x -y" in launch["10.0.0.1"]
    )
    assert "prior_map_x__hamilton__2" in launch["10.0.0.2"]
    # the new copy on topaz was marked with the prior's run id
    marks = [c for ip, c in ssh.calls if ip == "10.0.0.2" and ri.ORIGIN in c]
    assert marks


def test_launch_prior_phase_second_launch_is_a_noop():
    listings = {
        "willow": [_ident("map_x__hamilton", "x", origin={"run_id": "RID"})],
        "smaug": [_ident("prior_map_x__hamilton", "y", origin={"run_id": "RID"})],
        "topaz": [_ident("prior_map_x__hamilton", "z", origin={"run_id": "RID"})],
    }
    rsyncs = []
    rec = Recorder()
    ok = run_launch(
        _plan(),
        topo=TOPO,
        active_network="silvus",
        runtime_config=RUNTIME,
        output_root=OUT,
        callbacks=rec,
        deps=_deps(FakeSsh(), lambda *a, **k: rsyncs.append(a), listings),
    )
    assert ok and rsyncs == []
    assert rec.pct == {"smaug": 100, "topaz": 100}


def test_launch_prior_phase_falls_back_to_names_without_scan():
    rec = Recorder()
    ssh = FakeSsh({"echo OK": (0, "OK", "")})
    ok = run_launch(
        _plan(),
        topo=TOPO,
        active_network="silvus",
        runtime_config=RUNTIME,
        output_root=OUT,
        callbacks=rec,
        deps=_deps(ssh, lambda *a, **k: pytest.fail("no rsync expected"), {}),
    )
    assert ok and "prior map already present (prior_map_x__hamilton)" in rec.text()


def _legacy(name, digest=None, dsg=None, rid="legacy-1"):
    """A dir with only a fallback identity (no run_info, no origin)."""
    e = _ident(name, rid)
    e["identity"]["source"] = "fallback"
    e["manifest"] = {"digest": digest}
    e["dsg_sig"] = dsg
    return e


def _source(digest="D", dsg="100:5"):
    src = _legacy("map_x__hamilton", digest, dsg, rid="legacy-SRC")
    return src


def _launch(listings, rsyncs, ssh=None):
    rec = Recorder()
    ssh = ssh or FakeSsh()
    ok = run_launch(
        _plan(robots=["topaz"]),
        topo=TOPO,
        active_network="silvus",
        runtime_config=RUNTIME,
        output_root=OUT,
        callbacks=rec,
        deps=_deps(
            ssh, lambda *a, **k: rsyncs.append(a) or (True, "direct", "ok"), listings
        ),
    )
    return ok, rec, ssh


def test_unlabelled_copy_with_the_same_files_is_labelled_not_copied():
    # copied before copies recorded their origin: same files, own fallback id
    listings = {
        "willow": [_source()],
        "topaz": [_legacy("prior_map_x__hamilton", "D", "100:5", "legacy-T")],
    }
    rsyncs = []
    ok, rec, ssh = _launch(listings, rsyncs)
    assert ok, rec.text()
    assert rsyncs == [] and "labelled it, no transfer" in rec.text()
    marks = [c for ip, c in ssh.calls if ip == "10.0.0.2" and ri.ORIGIN in c]
    assert marks and "prior_map_x__hamilton/" in marks[0].replace("'", "")
    launch = [c for ip, c in ssh.calls if ip == "10.0.0.2" and "run-adt4" in c][0]
    assert "prior_map_x__hamilton " in launch.replace("'", "") + " "


def test_older_unlabelled_copy_is_updated_in_place():
    # same saved scene graph, other files differ (e.g. the source gained files since)
    listings = {
        "willow": [_source()],
        "topaz": [_legacy("prior_map_x__hamilton", "OLD", "100:5", "legacy-T")],
    }
    rsyncs = []
    ok, rec, ssh = _launch(listings, rsyncs)
    assert ok, rec.text()
    assert len(rsyncs) == 1 and rsyncs[0][5] == f"{OUT}/prior_map_x__hamilton"
    assert "updating it in place" in rec.text() and "__2" not in rec.text()
    assert [c for ip, c in ssh.calls if ip == "10.0.0.2" and ri.ORIGIN in c]


def test_unlabelled_dir_of_another_run_still_gets_a_new_name():
    listings = {
        "willow": [_source()],
        "topaz": [_legacy("prior_map_x__hamilton", "OTHER", "7:7", "legacy-T")],
    }
    rsyncs = []
    ok, rec, _ = _launch(listings, rsyncs)
    assert ok and len(rsyncs) == 1 and rsyncs[0][5] == f"{OUT}/prior_map_x__hamilton__2"
    assert "holds a different run" in rec.text()


def test_copies_record_the_source_manifest():
    listings = {"willow": [_source()], "topaz": []}
    rsyncs = []
    ok, rec, ssh = _launch(listings, rsyncs)
    mark = [c for ip, c in ssh.calls if ip == "10.0.0.2" and ri.ORIGIN in c][0]
    import base64
    import re as _re

    blob = _re.search(r"echo (\S+) \| base64 -d", mark).group(1)
    origin = json.loads(base64.b64decode(blob))
    assert origin["manifest_digest"] == "D" and origin["run_id"] == "legacy-SRC"
