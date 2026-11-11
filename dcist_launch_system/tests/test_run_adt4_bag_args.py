"""run-adt4: --bag-args reaches play_rosbag; ADT4_PRIOR_MAP is always exported."""

import importlib.machinery
import importlib.util
import pathlib
import shlex

import pytest
from click.testing import CliRunner

PKG = pathlib.Path(__file__).resolve().parents[1]


def _pane_cmd(pane):
    """A pane's command, whether panes are plain strings or zsh -ic pane shells."""
    if not isinstance(pane, dict):
        return pane
    argv = shlex.split(pane.get("shell", ""))
    if argv[:2] == ["zsh", "-ic"] and len(argv) == 3:
        for token in shlex.split(argv[2]):
            if token.startswith("ADT4_PANE_CMD="):
                return token[len("ADT4_PANE_CMD=") :]
    return pane.get("shell", "")


@pytest.fixture(scope="module")
def run_adt4():
    path = PKG / "bin" / "run-adt4"
    loader = importlib.machinery.SourceFileLoader("run_adt4", str(path))
    spec = importlib.util.spec_from_loader("run_adt4", loader)
    module = importlib.util.module_from_spec(spec)
    loader.exec_module(module)
    return module


def test_bag_args_are_appended_to_play_rosbag(run_adt4, tmp_path):
    info = run_adt4.SessionInfo(
        "hamilton",
        tmp_path / "out",
        platform_id="topaz",
        bag=tmp_path / "bag",
        bag_args=["--playback-duration", "180", "-f", ".*image.*"],
    )
    pane = _pane_cmd(run_adt4._make_bag_window(info)["panes"][0])
    assert pane.startswith(f"ros2 run ianvs play_rosbag {tmp_path / 'bag'} ")
    before_ros_args = pane.split(" --ros-args ")[0]
    assert before_ros_args.endswith(" --playback-duration 180 -f '.*image.*'")


def _run_main(run_adt4, monkeypatch, tmp_path, *args):
    captured = {}

    def fake_run_session(session_path, env, tmuxp_args, dry_run):
        captured["env"] = env
        return 0

    monkeypatch.setattr(run_adt4, "_run_session", fake_run_session)
    monkeypatch.delenv("ADT4_PRIOR_MAP", raising=False)
    # these tests are about the bag args, not the robot link
    monkeypatch.setenv("ADT4_FLEETLINK_MODE", "router")
    result = CliRunner().invoke(
        run_adt4.main,
        ["spot-default", "-n", "hamilton", "-c", "topaz", "-o", str(tmp_path / "out")]
        + ["-y", "--no-zenoh", *args],
    )
    assert result.exit_code == 0, result.output
    return captured["env"]


def test_prior_map_is_exported_empty_when_unset(run_adt4, monkeypatch, tmp_path):
    env = _run_main(run_adt4, monkeypatch, tmp_path)
    assert env["ADT4_PRIOR_MAP"] == ""


def test_prior_map_is_exported(run_adt4, monkeypatch, tmp_path):
    prior = tmp_path / "prior"
    prior.mkdir()
    env = _run_main(run_adt4, monkeypatch, tmp_path, "-p", str(prior))
    assert env["ADT4_PRIOR_MAP"] == str(prior)


def test_bag_args_option(run_adt4, monkeypatch, tmp_path):
    bag = tmp_path / "bag"
    bag.mkdir()
    seen = {}
    real = run_adt4._make_bag_window

    def spy(info):
        seen["bag_args"] = info.bag_args
        return real(info)

    monkeypatch.setattr(run_adt4, "_make_bag_window", spy)
    _run_main(
        run_adt4, monkeypatch, tmp_path, "--bag", str(bag),
        "--bag-args", "--playback-duration 180 -f '.*image.*'",
    )  # fmt: skip
    assert seen["bag_args"] == ["--playback-duration", "180", "-f", ".*image.*"]
