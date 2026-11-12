"""run-adt4: the windows it adds (zenoh, bag playback, recording) are process panes."""

import importlib.machinery
import importlib.util
import io
import os
import pathlib
import re
import shlex
import shutil
import subprocess
import time

import pytest
import yaml

PKG = pathlib.Path(__file__).resolve().parents[1]
SETUP = "source ${ADT4_WS}/install/setup.zsh"

needs_zsh = pytest.mark.skipif(shutil.which("zsh") is None, reason="zsh missing")


@pytest.fixture(scope="module")
def run_adt4():
    path = PKG / "bin" / "run-adt4"
    loader = importlib.machinery.SourceFileLoader("run_adt4", str(path))
    spec = importlib.util.spec_from_loader("run_adt4", loader)
    module = importlib.util.module_from_spec(spec)
    loader.exec_module(module)
    return module


def _script(shell):
    """The pane's script after its Ctrl-C preamble (asserted present)."""
    argv = shlex.split(shell)
    assert argv[:2] == ["zsh", "-ic"]
    assert argv[2].startswith("trap : INT; "), shell
    return argv[2][len("trap : INT; ") :]


def test_zenoh_window_runs_the_router(run_adt4):
    window = run_adt4._make_zenoh_window()
    script = _script(window["panes"][0]["shell"])
    assert script.startswith(f'{SETUP} && if ss -ltn | grep -q ":7447 "; then echo ')
    assert "; else ros2 run rmw_zenoh_cpp rmw_zenohd; fi; " in script
    assert script.endswith(" exec zsh")


def _fake_bin(tmp_path, ss_output):
    """A PATH dir with ``ss`` printing ``ss_output`` and ``ros2`` logging its argv."""
    bin_dir = tmp_path / "bin"
    bin_dir.mkdir()
    log = tmp_path / "ros2.log"
    (bin_dir / "ss").write_text(f"#!/bin/sh\ncat <<'EOF'\n{ss_output}\nEOF\n")
    (bin_dir / "ros2").write_text(f'#!/bin/sh\necho "$@" >> {log}\n')
    for f in bin_dir.iterdir():
        f.chmod(0o755)
    return bin_dir, log


SS_HEADER = "State  Recv-Q Send-Q Local Address:Port Peer Address:Port Process"


@needs_zsh
@pytest.mark.parametrize(
    "listening,port,runs",
    [
        ("LISTEN 0 4096 0.0.0.0:7447 0.0.0.0:*", 7447, False),
        ("LISTEN 0 4096 [::]:7447 [::]:*", 7447, False),
        ("LISTEN 0 4096 0.0.0.0:17447 0.0.0.0:*", 7447, True),
        ("LISTEN 0 4096 0.0.0.0:7447 0.0.0.0:*", 17447, True),
        ("", 7447, True),
    ],
)
def test_zenoh_pane_skips_a_running_router(run_adt4, tmp_path, listening, port, runs):
    """A router already on the port (adt4_console, another team's) is reported, not
    started a second time ("Address already in use")."""
    bin_dir, log = _fake_bin(tmp_path, f"{SS_HEADER}\n{listening}")
    pane = run_adt4._make_zenoh_window("router", port)["panes"][0]
    cmd = run_adt4.generate_configs.pane_command(pane)
    ret = subprocess.run(
        ["zsh", "-f", "-c", cmd],
        env={"PATH": f"{bin_dir}:{os.environ['PATH']}"},
        capture_output=True,
        text=True,
        timeout=30,
    )
    assert ret.returncode == 0, ret.stderr
    if runs:
        assert log.read_text() == "run rmw_zenoh_cpp rmw_zenohd\n"
        assert "already running" not in ret.stdout
    else:
        assert not log.exists()
        assert f"zenoh router already running on port {port}" in ret.stdout


def test_zenoh_port(run_adt4, tmp_path):
    topology = tmp_path / "topology.yaml"
    topology.write_text(yaml.safe_dump({"zenoh": {"port": 7500}}))
    assert run_adt4._zenoh_port({"ADT4_ZENOH_PORT": "17447"}) == 17447
    assert run_adt4._zenoh_port({"ADT4_TOPOLOGY": str(topology)}) == 7500
    missing = {"ADT4_TOPOLOGY": str(tmp_path / "missing.yaml")}
    assert run_adt4._zenoh_port(missing) == 7447


def test_expand_pane_shells_like_tmuxp(run_adt4, tmp_path):
    """$VARs in pane scripts come from run-adt4's environment, as tmuxp expanded typed
    commands, not from a tmux server another session started."""
    gc = run_adt4.generate_configs
    cmd = (
        "ros2 launch x conf_name:=$config robot_name:=${ADT4_ROBOT_NAME}"
        " seed=${ADT4_NEO4J_SEED:-true} other=$NOT_SET quote=$QUOTED"
    )
    session = tmp_path / "s.yaml"
    session.write_text(
        yaml.safe_dump(
            {
                "windows": [
                    {
                        "window_name": "w",
                        "panes": [{"shell": gc.pane_shell([SETUP], cmd)}],
                    }
                ]
            }
        )
    )
    env = {"config": "grstaps_repair", "ADT4_ROBOT_NAME": "willow", "QUOTED": "\"a b\"",
           "ADT4_WS": "/ws", "_adt4_zd": "never"}  # fmt: skip
    run_adt4._expand_pane_shells(session, env)
    shell = yaml.safe_load(session.read_text())["windows"][0]["panes"][0]["shell"]
    script = _script(shell)
    assert script.startswith(
        "source /ws/install/setup.zsh && ros2 launch x conf_name:=grstaps_repair "
        'robot_name:=willow seed=${ADT4_NEO4J_SEED:-true} other=$NOT_SET quote="a b"; '
    ), script
    assert "$_adt4_zd/.zshrc" in script  # the pane's own variables stay
    assert '_adt4_zd="/ws/src/awesome_dcist_t4/dcist_launch_system/' in script
    # the history entry is expanded the same way (it was the typed text before)
    assert gc.pane_command({"shell": shell}).startswith(
        "ros2 launch x conf_name:=grstaps"
    )
    ret = subprocess.run(["zsh", "-f", "-n", "-c", script], capture_output=True)
    assert ret.returncode == 0, ret.stderr


def test_session_file_uses_process_panes(run_adt4, tmp_path):
    session = tmp_path / "s.yaml"
    session.write_text(
        yaml.safe_dump(
            {
                "session_name": "adt4_system",
                "windows": [{"window_name": "w", "panes": [{"shell": "zsh -ic true"}]}],
            }
        )
    )
    info = run_adt4.SessionInfo(
        "willow",
        tmp_path / "out",
        platform_id="base_station",
        bag=tmp_path / "bag",
        trajectory=tmp_path / "traj.csv",
        record="base_station",
    )
    buf = io.StringIO()
    run_adt4._write_session_to_file(session, info, buf)
    contents = yaml.safe_load(buf.getvalue())
    names = [w["window_name"] for w in contents["windows"]]
    assert names[0] == "zenoh"
    assert "play_rosbag" in names and "recording" in names
    for window in contents["windows"]:
        assert "window_shell" not in window
        assert "shell_command_before" not in window
        for pane in window["panes"]:
            assert pane["shell"].startswith("zsh -ic ")
    bag = next(w for w in contents["windows"] if w["window_name"] == "play_rosbag")
    assert len(bag["panes"]) == 2
    play = _script(bag["panes"][0]["shell"])
    assert play.startswith(f"{SETUP} && ros2 run ianvs play_rosbag {tmp_path / 'bag'} ")
    # the trajectory pane runs on its own, not after the player
    traj = _script(bag["panes"][1]["shell"])
    assert traj.startswith(f"{SETUP} && ros2 run ianvs csv_to_tf ")
    assert "play_rosbag" not in traj
    rec = next(w for w in contents["windows"] if w["window_name"] == "recording")
    assert "record_topics" in _script(rec["panes"][0]["shell"])


needs_tmuxp = pytest.mark.skipif(
    not (shutil.which("tmuxp") and shutil.which("tmux") and shutil.which("zsh")),
    reason="tmux/tmuxp/zsh missing",
)


SEP = "<|>"


def _unescaped(command):
    """A command without quotes/backslashes: tmux's #{pane_start_command} quotes and
    escapes the command it started (the escaping differs by character)."""
    return re.sub(r'[\\"]', "", command)


def _wait(predicate, timeout=60.0):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return True
        time.sleep(0.25)
    return predicate()


@needs_tmuxp
def test_tmuxp_starts_every_pane_with_its_command(run_adt4, tmp_path):
    """Load a real generated session (+ run-adt4's windows) with tmuxp on a private
    tmux server: every pane is started as its command (nothing typed with send-keys),
    every `ros2 ...` pane really runs it, the prepared shutdown command waits on its
    prompt, and Ctrl-C on a command leaves the pane usable."""
    ws, home, bin_dir = tmp_path / "ws", tmp_path / "home", tmp_path / "bin"
    log = tmp_path / "ros2.log"
    for d in (ws / "install", home, bin_dir):
        d.mkdir(parents=True)
    zdot = ws / "src/awesome_dcist_t4/dcist_launch_system/config_generation/pane_shell"
    shutil.copytree(PKG / "config_generation" / "pane_shell", zdot)
    (ws / "install" / "setup.zsh").write_text(f"export PATH={bin_dir}:$PATH\n")
    (home / ".zshrc").write_text("PS1='pane> '\n")
    (bin_dir / "ros2").write_text(
        f'#!/bin/sh\necho "$TMUX_PANE $*" >> {log}\nexec sleep 300\n'
    )
    (bin_dir / "ros2").chmod(0o755)

    info = run_adt4.SessionInfo(
        "willow",
        tmp_path / "out",
        platform_id="base_station",
        bag=tmp_path / "bag",
        record="base_station",
        zenoh_port=1,  # nothing listens there: the router pane starts
    )
    session_file = tmp_path / "session.yaml"
    source = PKG / "tmux" / "autogenerated" / "base_station_web_repair-default.yaml"
    with session_file.open("w") as fout:
        run_adt4._write_session_to_file(source, info, fout)
    env = {
        "PATH": os.environ["PATH"],
        "HOME": str(home),
        "SHELL": shutil.which("zsh"),
        "TERM": "xterm-256color",
        "TMUX_TMPDIR": str(tmp_path),  # a private tmux server
        "ADT4_WS": str(ws),
        "ADT4_ROBOT_NAME": "willow",
        "ADT4_FLEETLINK_MODE": "router",
        "config": "default",
    }
    run_adt4._expand_pane_shells(session_file, env)
    session = yaml.safe_load(session_file.read_text())
    expected = {
        (w["window_name"], i): p["shell"]
        for w in session["windows"]
        for i, p in enumerate(w["panes"])
    }

    tmux = ["tmux", "-L", "panetest"]
    try:
        ret = subprocess.run(
            ["tmuxp", "load", "-L", "panetest", "-d", "-y", str(session_file)],
            env=env,
            capture_output=True,
            text=True,
            timeout=120,
        )
        assert ret.returncode == 0, ret.stderr

        def panes():
            out = subprocess.run(
                tmux + ["list-panes", "-s", "-t", session["session_name"], "-F",
                        SEP.join(["#{window_name}", "#{pane_index}", "#{pane_id}",
                                  "#{pane_start_command}"])],
                env=env, capture_output=True, text=True, check=True,
            ).stdout  # fmt: skip
            rows = [line.split(SEP, 3) for line in out.splitlines()]
            return {(w, int(i)): (pid, cmd) for w, i, pid, cmd in rows}

        started = panes()
        assert set(started) == set(expected)
        for key, shell in expected.items():
            assert _unescaped(started[key][1]) == _unescaped(shell), key

        pane_command = run_adt4.generate_configs.pane_command
        runs_ros2 = {
            key: started[key][0]
            for key, shell in expected.items()
            if pane_command({"shell": shell}).startswith("ros2 ")
            and key != ("core", 1)  # the prepared shutdown command
        }
        # the heracles publisher (`_seed=...; ... ros2 launch ...`), lost on willow
        assert pane_command({"shell": expected[("prior_dsg", 0)]}).startswith("_seed=")
        runs_ros2[("prior_dsg", 0)] = started[("prior_dsg", 0)][0]
        # tf_relay runs its launch in router mode
        runs_ros2[("tf_relay", 0)] = started[("tf_relay", 0)][0]

        def ros2_panes():
            text = log.read_text() if log.exists() else ""
            return {line.split(" ", 1)[0] for line in text.splitlines()}

        assert _wait(lambda: set(runs_ros2.values()) <= ros2_panes()), sorted(
            k for k, v in runs_ros2.items() if v not in ros2_panes()
        )
        # play_rosbag (lost to send-keys offline) and the guarded router
        ran = ros2_panes()
        assert started[("play_rosbag", 0)][0] in ran
        assert started[("zenoh", 0)][0] in ran  # its guard passed: port 1 is free

        def capture(key):
            return subprocess.run(
                tmux + ["capture-pane", "-p", "-t", started[key][0]],
                env=env, capture_output=True, text=True,
            ).stdout  # fmt: skip

        assert _wait(lambda: "ros2 service call" in capture(("core", 1)), 20)

        # Ctrl-C ends the command, not the pane
        target = started[("planning", 0)][0]
        subprocess.run(tmux + ["send-keys", "-t", target, "C-c"], env=env, check=True)
        assert _wait(lambda: "pane>" in capture(("planning", 0)), 20), capture(
            ("planning", 0)
        )
        assert ("planning", 0) in panes()
    finally:
        subprocess.run(tmux + ["kill-server"], env=env, capture_output=True)
