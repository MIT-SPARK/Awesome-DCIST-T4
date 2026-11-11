"""Optional console settings must reach panes on an existing tmux server."""

import importlib.machinery
import importlib.util
import json
import os
import pathlib
import shlex
import shutil
import subprocess
import time
import uuid

import pytest
import yaml


@pytest.fixture(scope="module")
def run_adt4():
    path = pathlib.Path(__file__).resolve().parents[1] / "bin" / "run-adt4"
    loader = importlib.machinery.SourceFileLoader("console_env_run_adt4", str(path))
    spec = importlib.util.spec_from_loader(loader.name, loader)
    module = importlib.util.module_from_spec(spec)
    loader.exec_module(module)
    return module


def write_session(run_adt4, tmp_path, pane="true", window="console", verbose=False):
    source = tmp_path / "source.yaml"
    source.write_text(
        yaml.safe_dump(
            {
                "session_name": "console-env-test",
                "options": {"default-command": "/bin/sh"},
                "windows": [{"window_name": window, "panes": [pane]}],
            }
        )
    )
    session = tmp_path / "session.yaml"
    info = run_adt4.SessionInfo("willow", tmp_path, zenoh=False)
    with session.open("w") as fout:
        run_adt4._write_session_to_file(source, info, fout, verbose=verbose)
    return session


def test_console_settings_forwarded_and_token_redacted(
    run_adt4, tmp_path, monkeypatch, capsys
):
    monkeypatch.setenv("ADT4_CONSOLE_TOKEN", "test-secret")
    monkeypatch.setenv("ADT4_CONSOLE_PORT", "18090")
    session = write_session(run_adt4, tmp_path, verbose=True)
    assert yaml.safe_load(session.read_text())["environment"] == {
        "ADT4_CONSOLE_TOKEN": "test-secret",
        "ADT4_CONSOLE_PORT": "18090",
    }
    output = capsys.readouterr().out
    assert "test-secret" not in output
    assert "<redacted>" in output


def test_unset_settings_clear_stale_server_values(run_adt4, tmp_path, monkeypatch):
    for key in run_adt4.CONSOLE_ENV_VARS:
        monkeypatch.delenv(key, raising=False)
    session = write_session(run_adt4, tmp_path)
    assert yaml.safe_load(session.read_text())["environment"] == {
        "ADT4_CONSOLE_TOKEN": "",
        "ADT4_CONSOLE_PORT": "",
    }


def test_robot_sessions_do_not_receive_console_credentials(
    run_adt4, tmp_path, monkeypatch
):
    monkeypatch.setenv("ADT4_CONSOLE_TOKEN", "test-secret")
    session = write_session(run_adt4, tmp_path, window="robot")
    env = yaml.safe_load(session.read_text()).get("environment", {})
    assert set(run_adt4.CONSOLE_ENV_VARS).isdisjoint(env)


@pytest.mark.skipif(
    not shutil.which("tmux") or not shutil.which("tmuxp"),
    reason="tmux and tmuxp required",
)
def test_console_pane_receives_settings_on_existing_server(
    run_adt4, tmp_path, monkeypatch
):
    socket = "console-env-test-" + uuid.uuid4().hex
    server_env = {
        k: v
        for k, v in os.environ.items()
        if k not in (*run_adt4.CONSOLE_ENV_VARS, "TMUX", "TMUX_PANE")
    }
    monkeypatch.setenv("ADT4_CONSOLE_TOKEN", "test-secret")
    monkeypatch.setenv("ADT4_CONSOLE_PORT", "18090")
    output = tmp_path / "pane.json"
    code = (
        "import os,json,pathlib; pathlib.Path("
        + repr(str(output))
        + ").write_text(json.dumps({k:os.environ.get(k) for k in "
        + repr(run_adt4.CONSOLE_ENV_VARS)
        + "}))"
    )
    pane = shlex.join(["/usr/bin/python3", "-c", code])
    session = write_session(run_adt4, tmp_path, pane=pane)
    try:
        subprocess.run(
            [
                "tmux",
                "-L",
                socket,
                "-f",
                "/dev/null",
                "new-session",
                "-d",
                "-s",
                "seed",
            ],
            env=server_env,
            check=True,
            capture_output=True,
            timeout=10,
        )
        subprocess.run(
            ["tmuxp", "load", "-d", "-L", socket, "-f", "/dev/null", str(session)],
            env={**server_env, **{k: os.environ[k] for k in run_adt4.CONSOLE_ENV_VARS}},
            check=True,
            capture_output=True,
            timeout=15,
        )
        deadline = time.monotonic() + 5
        while not output.exists() and time.monotonic() < deadline:
            time.sleep(0.02)
        assert output.exists(), "console pane did not write its environment"
        assert json.loads(output.read_text()) == {
            "ADT4_CONSOLE_TOKEN": "test-secret",
            "ADT4_CONSOLE_PORT": "18090",
        }
    finally:
        subprocess.run(
            ["tmux", "-L", socket, "kill-server"], capture_output=True, timeout=10
        )
