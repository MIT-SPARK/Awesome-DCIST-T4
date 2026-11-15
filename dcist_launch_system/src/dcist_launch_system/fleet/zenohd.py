"""Lifecycle of the fleet-managed local ``rmw_zenohd`` router.

Before the base-station session is launched, fleet tooling runs its own
zenohd so ROS discovery works for monitoring; it is stopped right before the
base station session takes port 7447.  ``state`` is a ``{"proc", "config"}``
dict owned by the caller (``TuiContext._fleet_zenoh`` or the web console).
"""

from __future__ import annotations

import os
import shlex
import shutil
import subprocess
import tempfile
import time

from dcist_launch_system.fleet_helpers import (
    generate_zenoh_endpoints,
    patch_zenoh_config_local,
)


def new_state() -> dict:
    return {"proc": None, "config": None}


def is_running(state: dict) -> bool:
    proc = state["proc"]
    return bool(proc) and proc.poll() is None


def stop(state: dict) -> None:
    """Terminate the fleet-managed zenohd so the base station session can take over."""
    proc = state["proc"]
    if proc and proc.poll() is None:
        proc.terminate()
        try:
            proc.wait(timeout=5)
        except subprocess.TimeoutExpired:
            proc.kill()
            proc.wait(timeout=2)
        time.sleep(1)  # allow port to be released (TIME_WAIT)
        state["proc"] = None


def start(
    state: dict, topo: dict, network: str, runtime_config: dict, local_ips
) -> tuple[bool, str]:
    """Start (or restart) the fleet-managed zenohd on the standard zenoh port.

    Returns (True, message) if started, (False, reason) otherwise.
    """
    if not shutil.which("ros2"):
        return False, "ros2 not found on PATH — source your ROS workspace first"

    # Don't start if one is already running under our control
    if state["proc"] and state["proc"].poll() is None:
        return True, "already running"

    try:
        zenoh_conf = topo.get("zenoh", {})
        port = zenoh_conf.get("port", 7447)

        # Determine this machine's role from topology
        local_machine = None
        for m in runtime_config.values():
            if m["ip"] in local_ips:
                local_machine = m
                break

        # Build connect endpoints from topology
        if local_machine:
            endpoints = generate_zenoh_endpoints(topo, network, local_machine["name"])
        else:
            endpoints = [
                f"tcp/{m['ip']}:{port}"
                for m in runtime_config.values()
                if m.get("role") == "base_station" and m.get("ip")
            ]

        # Clean up previous config
        if state["config"]:
            try:
                os.unlink(state["config"])
            except OSError:
                pass
            state["config"] = None

        config_path = patch_zenoh_config_local(endpoints)
        if not config_path:
            return False, "failed to patch zenoh config (template missing?)"

        state["config"] = config_path
        stderr_log = tempfile.NamedTemporaryFile(
            mode="w", prefix="fleet_zenohd_", suffix=".log", delete=False
        )
        # Source the ROS workspace before running zenohd — fleet-adt4 may
        # have been started without full ROS env (e.g. ros2 on PATH via
        # .zshrc but workspace not sourced). Use zsh login shell so
        # setup.zsh sources correctly and .zshrc env vars are available.
        shell_cmd = (
            f"source ~/dcist_ws/install/setup.zsh 2>/dev/null; "
            f"ZENOH_ROUTER_CONFIG_URI={shlex.quote(config_path)} "
            f"ros2 run rmw_zenoh_cpp rmw_zenohd"
        )
        zsh = shutil.which("zsh") or "zsh"
        state["proc"] = subprocess.Popen(
            [zsh, "-l", "-c", shell_cmd],
            stdout=subprocess.DEVNULL,
            stderr=stderr_log,
        )
        # Check it didn't die immediately
        time.sleep(1.0)
        if state["proc"].poll() is not None:
            rc = state["proc"].returncode
            state["proc"] = None
            stderr_log.close()
            try:
                with open(stderr_log.name) as f:
                    err_output = f.read().strip()[-500:]  # last 500 chars
            except Exception:
                err_output = ""
            detail = f"\n  stderr: {err_output}" if err_output else ""
            return (
                False,
                f"rmw_zenohd exited immediately (rc={rc}){detail}\n  config: {config_path}",
            )
        stderr_log.close()
        return True, f"started on port {port} with {len(endpoints)} endpoint(s)"
    except Exception as e:
        return False, f"error: {e}"


def cleanup(state: dict) -> None:
    """Stop the router and remove its temporary config file."""
    stop(state)
    if state["config"]:
        try:
            os.unlink(state["config"])
        except OSError:
            pass
        state["config"] = None
