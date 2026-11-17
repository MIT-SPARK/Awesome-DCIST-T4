"""trial_recorder: the per-trial recorder helper (bin/adt4-rec).

The pure parts run anywhere.  The end-to-end tests drive the real helper with
real tmux and a stand-in ``ros2`` on PATH; they need Linux (/proc) and tmux.
"""

import json
import os
import pathlib
import shutil
import subprocess
import sys
import textwrap
import time

import pytest

from dcist_launch_system import trial_recorder as tr

ADT4_REC = pathlib.Path(__file__).resolve().parents[1] / "bin" / "adt4-rec"
TRIAL = "trial_20261115T193012Z_door-test"


# --------------------------------------------------------------------- pure
def test_trial_of_reads_a_later_part_as_the_same_machine(tmp_path):
    assert tr.trial_of("/o/trial_x/euclid") == ("trial_x", "euclid")
    assert tr.trial_of("/o/trial_x/euclid_part3") == ("trial_x", "euclid")
    d, n = tr.part_dir(tmp_path, "euclid")
    assert (d.name, n) == ("euclid", 1)
    d.mkdir()
    assert tr.part_dir(tmp_path, "euclid")[0].name == "euclid_part2"
    (tmp_path / "euclid_part2").mkdir()
    assert tr.part_dir(tmp_path, "euclid") == (tmp_path / "euclid_part3", 3)


def test_trial_and_machine_names():
    assert tr.valid_trial(TRIAL)
    assert not tr.valid_trial("20261115_no_prefix")
    assert not tr.valid_trial("trial_../etc")
    assert not tr.valid_trial("trial_a/b")
    assert not tr.valid_trial("")
    assert tr.valid_name("euclid") and tr.valid_name("acl-gauss")
    assert not tr.valid_name("a b") and not tr.valid_name("../x")


def test_parse_topics_expands_dedupes_and_reports():
    text = textwrap.dedent(
        """\
        # sensors
        /${ADT4_ROBOT_NAME}/tf
        /$ADT4_ROBOT_NAME/odom   # trailing comment

        /${ADT4_ROBOT_NAME}/tf
        /euclid/omniplanner_node/compiled_plan_out
        """
    )
    topics, problems = tr.parse_topics(text, {"ADT4_ROBOT_NAME": "euclid"})
    assert topics == [
        "/euclid/tf",
        "/euclid/odom",
        "/euclid/omniplanner_node/compiled_plan_out",
    ]
    assert problems == []


def test_parse_topics_unset_variable_is_a_problem_not_a_topic():
    topics, problems = tr.parse_topics("/${NOPE}/tf\nnot a topic\n", {})
    assert topics == []
    assert len(problems) == 2 and "unset variable" in problems[0]


def test_repo_topic_lists_parse_cleanly():
    rec = pathlib.Path(__file__).resolve().parents[1] / "recording"
    for f in sorted(rec.glob("*_topics.txt")):
        topics, problems = tr.parse_topics(
            f.read_text(), {"ADT4_ROBOT_NAME": "euclid"}, ["euclid", "lewis"]
        )
        assert topics and not problems, (f.name, problems)
        assert not any("$" in t for t in topics)
    # a robot records its own plan; the base station every robot's
    spot, _ = tr.parse_topics(
        (rec / "spot_topics.txt").read_text(), {"ADT4_ROBOT_NAME": "lewis"}
    )
    assert "/lewis/omniplanner_node/compiled_plan_out" in spot
    assert not any(t.startswith(("/euclid/", "/hamilton/", "/gauss/")) for t in spot)
    base, _ = tr.parse_topics(
        (rec / "base_station_topics.txt").read_text(),
        {"ADT4_ROBOT_NAME": "willow"},
        ["euclid", "lewis"],
    )
    assert "/euclid/omniplanner_node/compiled_plan_out" in base
    assert "/lewis/omniplanner_node/compiled_plan_out" in base


def test_fleet_robot_lines_need_the_robots():
    text = "/${ADT4_FLEET_ROBOT}/plan\n/${ADT4_ROBOT_NAME}/tf\n"
    topics, problems = tr.parse_topics(text, {"ADT4_ROBOT_NAME": "willow"})
    assert topics == ["/willow/tf"] and "needs the fleet's robots" in problems[0]
    topics, problems = tr.parse_topics(text, {"ADT4_ROBOT_NAME": "willow"}, ["a", "b"])
    assert topics == ["/a/plan", "/b/plan", "/willow/tf"] and not problems


def test_map_identity_hashes_the_scene_graph(tmp_path):
    assert tr.map_identity("") is None
    assert tr.map_identity(tmp_path / "nope") == {
        "path": str(tmp_path / "nope"),
        "exists": False,
        "files": {},
    }
    prior = tmp_path / "prior_b45"
    (prior / "hydra" / "backend").mkdir(parents=True)
    (prior / "hydra" / "backend" / "dsg.json").write_text('{"nodes": []}')
    (prior / ".adt4_origin.json").write_text(
        json.dumps({"run_id": "r-1", "source_machine": "euclid"})
    )
    ident = tr.map_identity(prior)
    f = ident["files"]["hydra/backend/dsg.json"]
    import hashlib

    assert (
        f["sha256"] == hashlib.sha256(b'{"nodes": []}').hexdigest() and f["size"] == 13
    )
    assert ident["run_id"] == "r-1" and ident["source_machine"] == "euclid"
    assert "hydra/backend/dsg_with_mesh.json" not in ident["files"]


def test_shared_env_keeps_only_ros_settings():
    env = tr.parse_tmux_env(
        "ADT4_OUTPUT_DIR=/home/swarm/adt4_output/run1\nRMW_IMPLEMENTATION=rmw_zenoh_cpp\n"
        "ROS_DOMAIN_ID=7\n-UNSET_ONE\nHOME=/home/swarm\nsim_time=false\n"
    )
    assert env["HOME"] == "/home/swarm"
    assert tr.shared_env(env) == {
        "ADT4_OUTPUT_DIR": "/home/swarm/adt4_output/run1",
        "RMW_IMPLEMENTATION": "rmw_zenoh_cpp",
        "ROS_DOMAIN_ID": "7",
    }


def test_find_recorders_ours_and_others():
    rec = [
        "/usr/bin/python3",
        "/opt/ros/jazzy/bin/ros2",
        "bag",
        "record",
        "-o",
        "/x/t/m",
    ]
    procs = {
        1: {"ppid": 0, "argv": ["init"]},
        10: {"ppid": 1, "argv": ["zsh", "-ic", "..."]},  # our pane
        11: {"ppid": 10, "argv": rec},
        20: {"ppid": 1, "argv": ["zsh"]},  # the stack's recording pane
        21: {"ppid": 20, "argv": ["python3", "record_topics", "out", "f"]},
        22: {
            "ppid": 21,
            "argv": [
                "python3",
                "/opt/ros/jazzy/bin/ros2",
                "bag",
                "record",
                "-o",
                "out",
            ],
        },
        30: {"ppid": 1, "argv": ["ros2", "topic", "list"]},
    }
    ours, others = tr.find_recorders(procs, [10])
    assert ours == [11] and others == [22]
    assert tr.find_recorders(procs, []) == ([], [11, 22])
    assert tr.output_arg(rec) == "/x/t/m"


def test_pane_script_exports_sources_then_records():
    cmd = tr.record_command(
        "/out/t/m", ["/m/tf", "/m/odom"], ["--max-bag-duration", "60"]
    )
    cfg = tr.package_root() / "recording" / "mcap_zstd.yaml"
    assert cmd[:5] == ["ros2", "bag", "record", "-o", "/out/t/m"]
    assert cmd[-3:] == ["--topics", "/m/tf", "/m/odom"]
    script = tr.pane_script(
        {"ROS_DOMAIN_ID": "7", "X": "a b"}, "source ws/setup.zsh", cmd
    )
    lines = script.splitlines()
    assert lines[0] == "export ROS_DOMAIN_ID=7" and lines[1] == "export X='a b'"
    assert lines[2] == "source ws/setup.zsh"
    assert lines[3].startswith(
        f"ros2 bag record -o /out/t/m --storage-config-file {cfg} "
        "--max-bag-duration 60 --topics"
    )
    assert "exec" not in script  # the pane must outlive the recorder to show its error


METADATA = textwrap.dedent(
    """\
    rosbag2_bagfile_information:
      version: 9
      storage_identifier: mcap
      duration:
        nanoseconds: 12500000000
      starting_time:
        nanoseconds_since_epoch: 1763235012000000000
      message_count: 130
      topics_with_message_count:
        - topic_metadata:
            name: /euclid/tf
            type: tf2_msgs/msg/TFMessage
            serialization_format: cdr
          message_count: 125
        - topic_metadata:
            name: /euclid/odom
            type: nav_msgs/msg/Odometry
            serialization_format: cdr
          message_count: 5
        - topic_metadata:
            name: /euclid/fix
            type: sensor_msgs/msg/NavSatFix
            serialization_format: cdr
          message_count: 0
      compression_format: ""
    """
)


@pytest.mark.parametrize("with_yaml", [True, False])
def test_parse_metadata_counts(monkeypatch, with_yaml):
    if not with_yaml:
        monkeypatch.setitem(sys.modules, "yaml", None)  # import yaml -> ImportError
    else:
        pytest.importorskip("yaml")
    meta = tr.parse_metadata(METADATA)
    assert meta["message_count"] == 130
    assert meta["duration_s"] == pytest.approx(12.5)
    assert meta["topics"] == {"/euclid/tf": 125, "/euclid/odom": 5, "/euclid/fix": 0}
    wanted = ["/euclid/tf", "/euclid/fix", "/euclid/never"]
    assert tr.silent_topics(wanted, meta["topics"]) == ["/euclid/fix", "/euclid/never"]


def test_published_topics_ignores_subscribed_only():
    text = textwrap.dedent(
        """\
        Published topics:
         * /euclid/tf [tf2_msgs/msg/TFMessage] 2 publishers
         * /rosout [rcl_interfaces/msg/Log] 1 publisher

        Subscribed topics:
         * /euclid/tf [tf2_msgs/msg/TFMessage] 1 subscriber
         * /euclid/odom [nav_msgs/msg/Odometry] 1 subscriber
        """
    )
    assert tr.published_topics(text) == {"/euclid/tf", "/rosout"}


# ------------------------------------------------------------- end to end
FAKE_ROS2 = r'''#!/usr/bin/env python3
"""Stand-in for ros2: `bag record -o DIR --topics ...` and `topic list -v`."""
import os, signal, sys, time
a = sys.argv[1:]
if a[:2] == ["topic", "list"]:
    print("Published topics:\n * /husky/tf [tf2_msgs/msg/TFMessage] 1 publisher\n")
    print("Subscribed topics:\n * /husky/odom [nav_msgs/msg/Odometry] 1 subscriber")
    sys.exit(0)
assert a[:2] == ["bag", "record"], a
out = a[a.index("-o") + 1]
topics = a[a.index("--topics") + 1:]
if os.environ.get("FAKE_ROS2_FAIL"):
    print("[ERROR] [rosbag2]: could not open the storage plugin"); sys.exit(1)
os.makedirs(out)  # like rosbag2: fails when the directory exists
print("[INFO] [rosbag2_recorder]: Recording...", flush=True)
stop = []
signal.signal(signal.SIGINT, lambda *_: stop.append(1))
if os.environ.get("FAKE_ROS2_STUBBORN"):
    signal.signal(signal.SIGINT, signal.SIG_IGN)
n = 0
with open(os.path.join(out, os.path.basename(out) + "_0.mcap"), "wb") as f:
    while not stop:
        f.write(b"x" * 4096); f.flush(); n += 1
        time.sleep(0.1)
with open(os.path.join(out, "metadata.yaml"), "w") as f:
    f.write("rosbag2_bagfile_information:\n  duration:\n    nanoseconds: %d\n" % (n * 10**8))
    f.write("  message_count: %d\n  topics_with_message_count:\n" % n)
    for i, t in enumerate(topics):
        f.write("    - topic_metadata:\n        name: %s\n      message_count: %d\n" % (t, n if i == 0 else 0))
print("[INFO] [rosbag2_recorder]: Recording stopped", flush=True)
'''

needs_linux_tmux = pytest.mark.skipif(
    not (sys.platform.startswith("linux") and shutil.which("tmux")),
    reason="needs Linux /proc and tmux",
)


@pytest.fixture
def rig(tmp_path, monkeypatch):
    """A private tmux server, a fake ros2 on PATH, and an output root."""
    bindir = tmp_path / "bin"
    bindir.mkdir()
    ros2 = bindir / "ros2"
    ros2.write_text(FAKE_ROS2)
    ros2.chmod(0o755)
    topics = tmp_path / "topics.txt"
    topics.write_text("/${ADT4_ROBOT_NAME}/tf\n/${ADT4_ROBOT_NAME}/odom\n")
    env = {
        **os.environ,
        "PATH": f"{bindir}:{os.environ['PATH']}",
        "TMUX_TMPDIR": str(tmp_path),  # its own tmux server
        "ADT4_REC_SHELL": "bash -c",
        "ADT4_REC_SETUP": "",
    }
    env.pop("TMUX", None)
    root = tmp_path / "adt4_output"

    def run(*args, extra_env=None):
        r = subprocess.run(
            [sys.executable, str(ADT4_REC), "--output-root", str(root), *args],
            env={**env, **(extra_env or {})}, capture_output=True, text=True, timeout=120,
        )  # fmt: skip
        return r.returncode, json.loads(r.stdout.strip().splitlines()[-1])

    def tmux(*args):
        return subprocess.run(["tmux", *args], env=env, capture_output=True, text=True)

    yield type("Rig", (), {"run": staticmethod(run), "tmux": staticmethod(tmux),
                           "root": root, "topics": topics, "env": env})  # fmt: skip
    subprocess.run(["tmux", "-L", tr.REC_SOCKET, "kill-server"], env=env, capture_output=True)  # fmt: skip
    subprocess.run(["tmux", "kill-server"], env=env, capture_output=True)


def start_args(rig, trial=TRIAL):
    return ["start", "--trial", trial, "--machine", "husky", "--topics-file", str(rig.topics),
            "--description", "door test", "--base-time", "1763235012.5", "--growth-wait", "1"]  # fmt: skip


@needs_linux_tmux
def test_start_status_stop_round_trip(rig):
    rc, st = rig.run("status")
    assert rc == 0 and st["recording"] is False and st["free_bytes"] > 0

    # a stand-in stack session: its settings reach the recorder
    run_dir = rig.root / "run1"
    run_dir.mkdir(parents=True)
    (run_dir / "run_info.json").write_text(
        json.dumps({"run_id": "abc", "experiment": "run1"})
    )
    rig.tmux("new-session", "-d", "-s", "adt4_system", "-n", "status", "bash")
    rig.tmux("set-environment", "-t", "adt4_system", "ADT4_OUTPUT_DIR", str(run_dir))
    prior = rig.root / "prior_map"
    (prior / "hydra" / "backend").mkdir(parents=True)
    (prior / "hydra" / "backend" / "dsg.json").write_text("{}")
    rig.tmux("set-environment", "-t", "adt4_system", "ADT4_PRIOR_MAP", str(prior))
    rig.tmux("new-window", "-t", "adt4_system", "-n", "hydra", "bash")
    rig.tmux("send-keys", "-t", "adt4_system:hydra", "echo spam-every-second", "Enter")
    time.sleep(0.5)

    rc, out = rig.run(*start_args(rig))
    assert rc == 0, out
    assert out["recording"] and out["growing"] and out["topics"] == 2
    bag = rig.root / TRIAL / "husky"
    assert out["dir"] == str(bag)
    info = json.loads((bag / "trial.json").read_text())
    assert info["trial_id"] == TRIAL and info["description"] == "door test"
    assert info["topics"] == ["/husky/tf", "/husky/odom"]
    assert info["run"]["run_id"] == "abc" and info["stack_running"] is True
    assert info["prior_map"]["path"] == str(prior)
    assert len(info["prior_map"]["files"]["hydra/backend/dsg.json"]["sha256"]) == 64
    assert info["base_time_unix"] == 1763235012.5 and info["stopped_at"] is None

    rc, st = rig.run("status")
    assert st["recording"] and st["trial"] == TRIAL and st["machine"] == "husky"
    assert st["bytes"] > 0 and st["last_write_age_s"] < 5 and st["pid"] == out["pid"]

    # a second start is refused and changes nothing
    rc, again = rig.run(*start_args(rig, "trial_other"))
    assert rc == tr.BUSY and again["trial"] == TRIAL
    assert not (rig.root / "trial_other").exists()

    rc, ck = rig.run("check-topics")
    assert rc == 0 and ck["missing"] == ["/husky/odom"] and ck["published"] == 1

    # stopping another trial's id is refused
    rc, wrong = rig.run("stop", "--trial", "trial_other")
    assert rc == tr.FAILED and "not stopping" in wrong["error"]

    rc, sp = rig.run("stop", "--trial", TRIAL)
    assert rc == 0, sp
    assert sp["stopped"] and sp["metadata"] and sp["signals"] == ["SIGINT"]
    assert sp["silent_topics"] == ["/husky/odom"]
    info = json.loads((bag / "trial.json").read_text())
    assert info["stopped_at"] and info["topic_counts"]["/husky/tf"] > 0
    assert "Recording stopped" in (bag / "recorder.log").read_text()

    rc, st = rig.run("status")
    assert st["recording"] is False and st["session"] is False
    # the stack session was never touched
    assert rig.tmux("has-session", "-t", "=adt4_system").returncode == 0
    rc, sp = rig.run("stop")
    assert rc == 0 and sp["stopped"] is False


@needs_linux_tmux
def test_start_reports_a_recorder_that_fails(rig):
    rc, out = rig.run(
        *start_args(rig), "--start-wait", "4", extra_env={"FAKE_ROS2_FAIL": "1"}
    )
    assert rc == tr.FAILED and "did not start" in out["error"]
    assert "storage plugin" in out["pane"]
    # and the next start is not blocked by the dead session
    rc, out = rig.run(*start_args(rig))
    assert rc == 0, out
    rig.run("stop")


@needs_linux_tmux
def test_never_records_over_an_existing_directory(rig):
    """A start whose bag directory is there already (the recorder died and the
    console retries) records into the next part beside it."""
    old = rig.root / TRIAL / "husky"
    old.mkdir(parents=True)
    (old / "husky_0.mcap").write_bytes(b"first part")
    rc, out = rig.run(*start_args(rig))
    assert rc == 0, out
    part2 = rig.root / TRIAL / "husky_part2"
    assert out["dir"] == str(part2) and out["part"] == 2
    rc, st = rig.run("status")
    assert st["recording"] and (st["trial"], st["machine"]) == (TRIAL, "husky")
    rc, sp = rig.run("stop", "--trial", TRIAL)
    assert rc == 0 and sp["metadata"] is True and sp["machine"] == "husky"
    info = json.loads((part2 / "trial.json").read_text())
    assert (info["machine"], info["part"]) == ("husky", 2)
    assert (old / "husky_0.mcap").read_bytes() == b"first part"
    assert sorted(p.name for p in (rig.root / TRIAL).iterdir()) == ["husky", "husky_part2"]  # fmt: skip
    # and once more: part 3
    rc, out = rig.run(*start_args(rig))
    assert rc == 0 and out["part"] == 3, out
    rig.run("stop")


@needs_linux_tmux
def test_the_recorder_outlives_the_stacks_tmux_server(rig):
    """``tmux kill-server`` on the machine (how a stack is often torn down)
    must not take a recording with it, and ``tmux attach`` must not find it."""
    rig.tmux("new-session", "-d", "-s", "adt4_system", "-n", "status", "bash")
    rc, out = rig.run(*start_args(rig))
    assert rc == 0, out
    assert rig.tmux("has-session", "-t", "=adt4_rec").returncode != 0
    assert "adt4_rec" not in rig.tmux("list-sessions").stdout
    assert rig.tmux("kill-server").returncode == 0
    rc, st = rig.run("status")
    assert st["recording"] is True and st["stack_running"] is False
    rc, sp = rig.run("stop")
    assert rc == 0 and sp["stopped"] is True and sp["metadata"] is True
    assert rig.run("status")[1]["session"] is False


@needs_linux_tmux
def test_a_recorder_left_on_the_default_server_is_still_ours(rig):
    """One started before the recorder had its own server: seen and stopped."""
    bag = rig.root / TRIAL / "husky"
    bag.parent.mkdir(parents=True)
    rig.tmux("new-session", "-d", "-s", "adt4_rec", "bash", "-c",
             f"ros2 bag record -o {bag} --topics /husky/tf; sleep 30")  # fmt: skip
    for _ in range(40):
        rc, st = rig.run("status")
        if st["recording"]:
            break
        time.sleep(0.25)
    assert st["recording"] is True and st["dir"] == str(bag)
    rc, sp = rig.run("stop")
    assert rc == 0 and sp["stopped"] is True
    assert rig.tmux("has-session", "-t", "=adt4_rec").returncode != 0


@needs_linux_tmux
def test_stubborn_recorder_is_left_running_unless_forced(rig):
    rc, out = rig.run(*start_args(rig), extra_env={"FAKE_ROS2_STUBBORN": "1"})
    assert rc == 0, out
    rc, sp = rig.run("stop", "--wait", "2")
    assert rc == tr.STILL_RUNNING and sp["recording"] and sp["signals"] == ["SIGINT"]
    assert rig.run("status")[1]["recording"] is True
    t0 = time.monotonic()
    rc, sp = rig.run("stop", "--wait", "2", "--force")
    assert rc == 0 and sp["signals"] == ["SIGINT", "SIGINT", "SIGTERM"]
    assert sp["metadata"] is False  # killed before it could close the bag: reported
    assert time.monotonic() - t0 < 60


def test_compression_is_on_by_default_and_can_be_switched_off():
    cfg = tr.package_root() / "recording" / "mcap_zstd.yaml"
    assert cfg.is_file() and "compression: Zstd" in cfg.read_text()
    assert tr.compression_args({}) == ["--storage-config-file", str(cfg)]
    assert tr.compression_args({"ADT4_REC_COMPRESSION": "none"}) == []


def test_commands_run_without_the_callers_terminal():
    # a child in its own session cannot take the caller's terminal (zsh -i did,
    # and Ctrl-C stopped reaching the console)
    code = "import os; print(os.getsid(0) == os.getpid(), os.getsid(0))"
    rc, out, _ = tr._run([sys.executable, "-c", code])
    own, sid = out.split()
    assert rc == 0 and own == "True" and int(sid) != os.getsid(0)


def test_run_listing_records_what_the_run_folder_held(tmp_path):
    run = tmp_path / "exp_a"
    (run / "hydra" / "backend").mkdir(parents=True)
    (run / "hydra" / "backend" / "dsg.json").write_text("graph")
    (run / "log.txt").write_text("log")
    os.symlink(run / "log.txt", run / "link")
    got = tr.run_listing(str(run))
    assert [(f["path"], f["size"]) for f in got["files"]] == [
        ("log.txt", 3),
        ("hydra/backend/dsg.json", 5),
    ]
    assert got["bytes"] == 8 and got["truncated"] is False
    assert all(f["mtime"] > 0 for f in got["files"])
    # a huge folder is cut, and says so; the total still counts everything
    cut = tr.run_listing(str(run), limit=1)
    assert len(cut["files"]) == 1 and cut["truncated"] is True and cut["bytes"] == 8
    # no run folder: nothing, and no error
    assert tr.run_listing(None) is None
    assert tr.run_listing(str(tmp_path / "nope")) is None
