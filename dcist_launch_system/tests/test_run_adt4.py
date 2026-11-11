"""run-adt4: the windows it adds (zenoh, bag playback, recording) are process panes."""

import importlib.machinery
import importlib.util
import io
import pathlib
import shlex

import pytest
import yaml

PKG = pathlib.Path(__file__).resolve().parents[1]
SETUP = "source ${ADT4_WS}/install/setup.zsh"


@pytest.fixture(scope="module")
def run_adt4():
    path = PKG / "bin" / "run-adt4"
    loader = importlib.machinery.SourceFileLoader("run_adt4", str(path))
    spec = importlib.util.spec_from_loader("run_adt4", loader)
    module = importlib.util.module_from_spec(spec)
    loader.exec_module(module)
    return module


def _script(shell):
    argv = shlex.split(shell)
    assert argv[:2] == ["zsh", "-ic"]
    return argv[2]


def test_zenoh_window_runs_the_router(run_adt4):
    window = run_adt4._make_zenoh_window()
    script = _script(window["panes"][0]["shell"])
    assert script.startswith(f"{SETUP} && ros2 run rmw_zenoh_cpp rmw_zenohd; ")
    assert script.endswith(" exec zsh")


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
