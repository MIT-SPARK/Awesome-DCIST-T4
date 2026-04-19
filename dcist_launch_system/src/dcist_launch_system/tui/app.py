"""FleetApp — main Textual application, zenohd lifecycle, and TUI entry point."""

from __future__ import annotations

import os
import shlex
import shutil
import subprocess
import sys
import tempfile
import threading
import time

import click
from textual.app import App, ComposeResult
from textual.binding import Binding
from textual.containers import Vertical
from textual.widgets import DataTable, Footer, Header, Label, RichLog, Rule

from dcist_launch_system.fleet_helpers import (
    detect_network,
    filter_reachable,
    generate_zenoh_endpoints,
    get_local_ips,
    load_topology,
    patch_zenoh_config_local,
    resolve_machines,
)
from dcist_launch_system.tui.context import TuiContext
from dcist_launch_system.tui.screens import (
    ConfigScreen,
    LaunchScreen,
    MapsScreen,
    MonitorScreen,
    ZenohScreen,
)


def _stop_fleet_zenohd(ctx: TuiContext) -> None:
    """Terminate the fleet-managed zenohd so the base station session can take over."""
    proc = ctx._fleet_zenoh["proc"]
    if proc and proc.poll() is None:
        proc.terminate()
        try:
            proc.wait(timeout=5)
        except subprocess.TimeoutExpired:
            proc.kill()
            proc.wait(timeout=2)
        time.sleep(1)  # allow port to be released (TIME_WAIT)
        ctx._fleet_zenoh["proc"] = None


def _start_fleet_zenohd(ctx: TuiContext) -> tuple[bool, str]:
    """Start (or restart) the fleet-managed zenohd on the standard zenoh port.

    Returns (True, message) if started, (False, reason) otherwise.
    """
    if not shutil.which("ros2"):
        return False, "ros2 not found on PATH — source your ROS workspace first"

    # Don't start if one is already running under our control
    if ctx._fleet_zenoh["proc"] and ctx._fleet_zenoh["proc"].poll() is None:
        return True, "already running"

    try:
        local_ips = ctx._local_ips
        zenoh_conf = ctx.topo.get("zenoh", {})
        port = zenoh_conf.get("port", 7447)

        # Determine this machine's role from topology
        local_machine = None
        for m in ctx.runtime_config.values():
            if m["ip"] in local_ips:
                local_machine = m
                break

        # Build connect endpoints from topology
        if local_machine:
            endpoints = generate_zenoh_endpoints(
                ctx.topo, ctx.active_network, local_machine["name"]
            )
        else:
            endpoints = [
                f"tcp/{m['ip']}:{port}"
                for m in ctx.runtime_config.values()
                if m.get("role") == "base_station" and m.get("ip")
            ]

        # Clean up previous config
        if ctx._fleet_zenoh["config"]:
            try:
                os.unlink(ctx._fleet_zenoh["config"])
            except OSError:
                pass
            ctx._fleet_zenoh["config"] = None

        config_path = patch_zenoh_config_local(endpoints)
        if not config_path:
            return False, "failed to patch zenoh config (template missing?)"

        ctx._fleet_zenoh["config"] = config_path
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
        ctx._fleet_zenoh["proc"] = subprocess.Popen(
            [zsh, "-l", "-c", shell_cmd],
            stdout=subprocess.DEVNULL,
            stderr=stderr_log,
        )
        # Check it didn't die immediately
        time.sleep(1.0)
        if ctx._fleet_zenoh["proc"].poll() is not None:
            rc = ctx._fleet_zenoh["proc"].returncode
            ctx._fleet_zenoh["proc"] = None
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


class FleetApp(App):
    CSS = """
    Screen {
        background: $surface;
    }
    #status_table {
        height: auto;
        max-height: 50%;
    }
    #log {
        height: auto;
        min-height: 5;
    }
    ModalScreen {
        align: center middle;
    }
    """

    BINDINGS = [
        Binding("s", "refresh_status", "Recheck"),
        Binding("l", "show_launch", "Launch"),
        Binding("b", "show_maps", "Browse Maps"),
        Binding("c", "show_config", "Config"),
        Binding("o", "show_monitor", "Monitor"),
        Binding("z", "show_zenoh", "Zenoh"),
        Binding("q", "quit", "Quit"),
    ]

    def __init__(self, ctx: TuiContext):
        super().__init__()
        self.ctx = ctx

    def compose(self) -> ComposeResult:
        yield Header()
        yield Vertical(
            Label(f"[bold]fleet-adt4[/]  network: [cyan]{self.ctx.active_network}[/]"),
            Rule(),
            DataTable(id="status_table"),
            Rule(),
            RichLog(id="log", highlight=True, markup=True, max_lines=100),
        )
        yield Footer()

    def on_mount(self):
        self.title = "fleet-adt4"
        self.sub_title = f"network: {self.ctx.active_network}"
        table = self.query_one("#status_table", DataTable)
        table.add_column("Machine", key="machine")
        table.add_column("Role", key="role")
        table.add_column("Platform", key="platform")
        table.add_column("IP", key="ip")
        table.add_column("Ping", key="ping")
        table.add_column("SSH", key="ssh")
        table.add_column("Status", key="status")
        self.action_refresh_status()

    def action_refresh_status(self):
        log = self.query_one("#log", RichLog)
        log.write("[dim]Checking fleet connectivity (ping + SSH)...[/]")

        def do_check():
            machines = list(self.ctx.runtime_config.values())
            reachable, unreachable = filter_reachable(machines)
            return reachable + unreachable

        def on_done(all_machines):
            table = self.query_one("#status_table", DataTable)
            table.clear()
            n_online = 0
            for m in sorted(all_machines, key=lambda x: x["name"]):
                ping_str = "OK" if m.get("ping") else "FAIL"
                ssh_str = "OK" if m.get("ssh") else "FAIL"
                if m.get("online"):
                    status_str = "ONLINE"
                    n_online += 1
                elif m.get("ping"):
                    status_str = "NO SSH"
                else:
                    status_str = "OFFLINE"
                table.add_row(
                    m["name"],
                    m["role"],
                    self.ctx.runtime_config[m["name"]].get("platform_id", "---"),
                    m["ip"],
                    ping_str,
                    ssh_str,
                    status_str,
                )
                # Update runtime_config with reachability
                self.ctx.runtime_config[m["name"]].update(
                    {
                        "ping": m.get("ping", False),
                        "ssh": m.get("ssh", False),
                        "online": m.get("online", False),
                    }
                )
            log.write(f"[green]{n_online}/{len(all_machines)} machines online.[/]")

        def bg():
            results = do_check()
            self.app.call_from_thread(on_done, results)

        log = self.query_one("#log", RichLog)
        threading.Thread(target=bg, daemon=True).start()

    def action_show_maps(self):
        self.push_screen(MapsScreen(self.ctx))

    def action_show_launch(self):
        # Create fresh each time — Textual doesn't preserve select()
        # state across dismiss/re-push. The mode is re-applied on mount.
        mode = getattr(self, "_last_launch_mode", None)
        self.push_screen(
            LaunchScreen(
                self.ctx,
                stop_zenohd=lambda: _stop_fleet_zenohd(self.ctx),
                mode=mode,
            )
        )

    def action_show_config(self):
        self.push_screen(ConfigScreen(self.ctx))

    def action_show_monitor(self):
        self.push_screen(
            MonitorScreen(
                self.ctx,
                start_zenohd=lambda: _start_fleet_zenohd(self.ctx),
                stop_zenohd=lambda: _stop_fleet_zenohd(self.ctx),
            )
        )

    def action_show_zenoh(self):
        self.push_screen(ZenohScreen(self.ctx))


def _launch_tui(
    network: str | None,
    topology_path: str | None,
    output_root: str,
    radio_type: str = "auto",
) -> None:
    """Build TuiContext and run FleetApp."""
    try:
        import textual  # noqa: F401
    except ImportError:
        click.secho(
            "textual not installed. Install with: pip install textual\n"
            "Falling back to CLI mode. Use subcommands (status, maps, etc.).",
            fg="yellow",
        )
        return

    topo = load_topology(topology_path)
    active_network = network or detect_network(topo)
    if not active_network:
        click.secho("Could not detect network. Use --network.", fg="red")
        sys.exit(1)

    all_machines = resolve_machines(topo, active_network)
    runtime_config = {m["name"]: dict(m) for m in all_machines}
    local_ips = set(get_local_ips())

    ctx = TuiContext(
        topo=topo,
        active_network=active_network,
        topology_path=topology_path,
        output_root=output_root,
        radio_type_override=radio_type,
        runtime_config=runtime_config,
        _local_ips=local_ips,
    )

    try:
        app = FleetApp(ctx)
        app.run()
    finally:
        _stop_fleet_zenohd(ctx)
        if ctx._fleet_zenoh["config"]:
            try:
                os.unlink(ctx._fleet_zenoh["config"])
            except OSError:
                pass
