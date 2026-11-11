"""install/python_setup.bash builds the console UI only with a supported Node and fails when the build fails."""

import os
import pathlib
import shlex
import shutil
import subprocess

import pytest

ROOT = pathlib.Path(__file__).resolve().parents[2]
SETUP = ROOT / "install" / "python_setup.bash"


def console_build_env(tmp_path, failure=None, node_script="exit 0"):
    frontend = tmp_path / "ws/src/awesome_dcist_t4/adt4_console/frontend"
    frontend.mkdir(parents=True)
    (frontend / "package.json").write_text("{}")
    bin_dir = tmp_path / "bin"
    bin_dir.mkdir()
    log = tmp_path / "npm.log"
    scripts = {
        "node": node_script,
        "npm": 'echo "$*" >> "$NPM_LOG"\n[ "$*" != "$NPM_FAIL" ] || exit 42',
    }
    for name, body in scripts.items():
        script = bin_dir / name
        script.write_text("#!/bin/sh\n" + body + "\n")
        script.chmod(0o755)
    env = {
        **os.environ,
        "PATH": str(bin_dir) + os.pathsep + os.environ["PATH"],
        "ADT4_WS": str(tmp_path / "ws"),
        "ADT4_ENV": str(tmp_path / "env"),
        "NPM_LOG": str(log),
        "NPM_FAIL": failure or "",
    }
    return env, log


@pytest.mark.parametrize("failure", [None, "ci", "run build"])
def test_installer_propagates_console_build_failure(tmp_path, failure):
    env, log = console_build_env(tmp_path, failure)
    result = subprocess.run(
        ["bash", str(SETUP), "--no-roman", "--no-spark"],
        env=env,
        capture_output=True,
        text=True,
    )
    assert (result.returncode == 0) == (failure is None), result.stderr
    assert log.read_text().splitlines() == (
        ["ci"] if failure == "ci" else ["ci", "run build"]
    )
    if failure:
        assert "Console UI build failed" in result.stderr


@pytest.mark.parametrize(
    "version, supported",
    [
        ("20.18.0", False),
        ("20.19.0", True),
        ("21.7.0", False),
        ("22.11.0", False),
        ("22.12.0", True),
        ("24.14.0", True),
    ],
)
def test_node_guard_matches_frontend_engine(tmp_path, version, supported):
    node = shutil.which("node")
    if node is None:
        pytest.skip("Node is needed to evaluate the installer version check")
    env, log = console_build_env(
        tmp_path,
        node_script=(
            f"exec {shlex.quote(node)} -e "
            '\'Object.defineProperty(process.versions, "node", '
            '{value: process.env.TEST_NODE_VERSION}); eval(process.argv[1]);\' "$2"'
        ),
    )
    env["TEST_NODE_VERSION"] = version
    result = subprocess.run(
        ["bash", str(SETUP), "--no-roman", "--no-spark"],
        env=env,
        capture_output=True,
        text=True,
    )
    assert result.returncode == 0, result.stderr
    assert log.exists() == supported
    if supported:
        assert log.read_text().splitlines() == ["ci", "run build"]
    else:
        assert "Skipping the console UI build: it needs Node" in result.stdout
