"""Monitor screen — runtime fleet status."""
from __future__ import annotations

import shlex
import threading

from textual.app import ComposeResult
from textual.binding import Binding
from textual.containers import Horizontal, Vertical, VerticalScroll
from textual.screen import ModalScreen
from textual.widgets import (
    Button,
    DataTable,
    Footer,
    Input,
    Label,
    ProgressBar,
    RichLog,
    Rule,
    SelectionList,
    Static,
    Tree,
)
from dcist_launch_system.fleet_helpers import (
    _quote_path,
    check_silvus_route,
    check_zenoh_config,
    check_zenoh_port,
    compute_robot_readiness,
    deploy_zenoh_config,
    filter_reachable,
    generate_namespaced_rviz,
    generate_zenoh_endpoints,
    get_remote_status,
    get_ros_node_status,
    get_silvus_link_quality,
    get_silvus_radio_details,
    hash_remote_experiment,
    list_remote_experiments,
    NodeStatusPoller,
    rsync_transfer,
    run_parallel,
    send_tmux_keys,
    ssh_cmd,
    check_iperf3,
    run_iperf3_test,
)
from dcist_launch_system.tui.context import TuiContext

import json

from rich.table import Table

class MonitorScreen(ModalScreen):
    BINDINGS = [
        Binding("escape", "dismiss", "Back", priority=True),
        Binding("r", "refresh", "Refresh", priority=True),
        Binding("n", "refresh_nodes", "Node Status", priority=True),
        Binding("k", "stop_save", "Stop & Save", priority=True),
        Binding("enter", "drill_down", "Details", priority=True),
        Binding("v", "gen_rviz", "RViz Config", priority=True),
        Binding("z", "check_zenoh", "Zenoh/Radio", priority=True),
        Binding("f3", "check_silvus_routes", "Check Routes [F3]", priority=True),
        Binding("f4", "silvus_detail", "Radio Detail [F4]", priority=True),
        Binding("f2", "start_zenohd", "Start Zenohd [F2]", priority=True),
        Binding("w", "bandwidth_test", "Bandwidth [w]", priority=True),
    ]

    def __init__(self, ctx: TuiContext, start_zenohd=None, stop_zenohd=None):
        super().__init__()
        self.ctx = ctx
        self._start_zenohd = start_zenohd
        self._stop_zenohd = stop_zenohd

    def compose(self) -> ComposeResult:
        yield VerticalScroll(
            Label("[bold]Fleet Monitor[/]"),
            Rule(),
            Label("[bold]System Status[/] (r=refresh)"),
            Static(id="monitor_table"),
            Rule(),
            Label("[bold]Node Readiness[/] (n=refresh, Enter=details, Esc=back, k=stop & save)"),
            DataTable(id="node_summary_table"),
            Static(id="node_detail_view"),
            Rule(),
            RichLog(id="monitor_log", highlight=True, markup=True, max_lines=30),
        )
        yield Footer()

    def _build_status_table(self, results=None):
        """Build a Rich Table from system status results."""
        t = Table(box=None, expand=True, pad_edge=False)
        t.add_column("Machine", no_wrap=True, min_width=8)
        t.add_column("IP", no_wrap=True, max_width=16)
        t.add_column("Tmux", no_wrap=False, max_width=20)
        t.add_column("ROS2", no_wrap=True, max_width=5)
        t.add_column("Load 1/5/15m", no_wrap=False, max_width=16)
        t.add_column("Mem", no_wrap=True, max_width=12)
        t.add_column("GPU", no_wrap=False, max_width=18)
        t.add_column("Disk", no_wrap=False, max_width=20)
        t.add_column("Bat", no_wrap=True, max_width=18)
        t.add_column("Radio", no_wrap=True, max_width=14)
        if results:
            for m, st in sorted(results, key=lambda x: x[0]["name"]):
                ip = m.get("ip", "")
                if isinstance(st, Exception):
                    t.add_row(m["name"], ip, str(st), "", "", "", "", "", "", "")
                else:
                    t.add_row(
                        m["name"],
                        ip,
                        st["tmux_sessions"],
                        st["ros2_procs"],
                        st["load"],
                        st.get("mem", "N/A"),
                        st.get("gpu", "N/A"),
                        st["disk"],
                        st.get("battery", "N/A"),
                        st.get("radio", "N/A"),
                    )
        return t

    def on_mount(self):
        self._node_status = {}  # robot_name -> [node dicts]

        # Render empty table initially
        widget = self.query_one("#monitor_table", Static)
        widget.update(self._build_status_table())

        node_table = self.query_one("#node_summary_table", DataTable)
        node_table.add_column("Robot", key="robot")
        node_table.add_column("Readiness", key="readiness")
        node_table.add_column("Nodes", key="nodes")
        node_table.add_column("Issues", key="issues")
        node_table.cursor_type = "row"

        self._node_poller = NodeStatusPoller()
        self._node_poller.start()
        self.action_refresh()
        # Read cached poller data every 1s (instant — just reads memory)
        self.set_interval(1, self._update_node_table)

    def action_refresh(self):
        log = self.query_one("#monitor_log", RichLog)
        log.write("[dim]Refreshing system status...[/]")

        def do_fetch():
            machines = [m for m in self.ctx.runtime_config.values() if m.get("online", False)]
            sys_radios = self.ctx.topo.get("silvus_radios", {}).get("radios", {})
            silvus_ips = [r["mgmt_ip"] for r in sys_radios.values() if "mgmt_ip" in r]

            def fetch(m):
                return get_remote_status(m["user"], m["ip"], silvus_ips=silvus_ips)

            return run_parallel(fetch, machines)

        def on_done(results):
            widget = self.query_one("#monitor_table", Static)
            widget.update(self._build_status_table(results))
            log.write("[green]System status updated.[/]")

        def bg():
            results = do_fetch()
            self.app.call_from_thread(on_done, results)

        threading.Thread(target=bg, daemon=True).start()

    def action_refresh_nodes(self):
        """Manual refresh — restart poller if dead, show status."""
        log = self.query_one("#monitor_log", RichLog)
        if not self._node_poller.is_running():
            ok, msg = self._node_poller.start()
            log.write(f"[dim]Node poller: {msg}[/]")
        err = self._node_poller.get_error()
        if err:
            log.write(f"[dim]ros2 stderr: {err}[/]")
        self._update_node_table()
        status = self._node_status
        if status:
            log.write(f"[green]Node status: {len(status)} machine(s)[/]")
        else:
            log.write("[yellow]No node status yet — poller may still be starting.[/]")

    def _update_node_table(self):
        """Read latest from persistent poller and refresh the table.

        Uses update_cell to avoid clearing/rebuilding which resets cursor.
        """
        status = self._node_poller.get_status()
        self._node_status = status
        node_table = self.query_one("#node_summary_table", DataTable)

        if not status:
            return

        # Build new row data
        new_rows = {}
        for robot_name in sorted(status):
            nodes = status[robot_name]
            color, label = compute_robot_readiness(nodes)
            n_total = len(nodes)
            n_nominal = sum(1 for n in nodes if n["status"] == _STATUS_NOMINAL)
            issues = []
            for n in nodes:
                if n["status"] != _STATUS_NOMINAL:
                    sname = _STATUS_NAMES.get(n["status"], "?")
                    issues.append(f"{n['nickname']}={sname}")
            new_rows[robot_name] = (
                f"[{color}]{label}[/{color}]",
                f"{n_nominal}/{n_total}",
                ", ".join(issues) if issues else "none",
            )

        # Check if rows changed — update in place if possible
        existing_keys = set()
        try:
            from textual.coordinate import Coordinate
            for i in range(node_table.row_count):
                existing_keys.add(str(node_table.get_cell_at(Coordinate(i, 0))))
        except Exception:
            existing_keys = set()

        if existing_keys == set(new_rows.keys()):
            # Same robots — update cells in place (no cursor reset)
            for i, robot_name in enumerate(sorted(new_rows)):
                readiness, node_count, issue_str = new_rows[robot_name]
                try:
                    from textual.coordinate import Coordinate
                    node_table.update_cell_at(Coordinate(i, 1), readiness)
                    node_table.update_cell_at(Coordinate(i, 2), node_count)
                    node_table.update_cell_at(Coordinate(i, 3), issue_str)
                except Exception:
                    break
        else:
            # Robot set changed — rebuild
            self._rebuilding_table = True
            node_table.clear()
            for robot_name in sorted(new_rows):
                readiness, node_count, issue_str = new_rows[robot_name]
                node_table.add_row(robot_name, readiness, node_count, issue_str, key=robot_name)
            self._rebuilding_table = False

        # Also refresh the detail view if one is showing
        self._show_node_detail()

    def on_data_table_cursor_moved(self, event):
        """Update detail view when cursor moves via arrow keys or click."""
        if event.data_table.id == "node_summary_table" and not getattr(self, "_rebuilding_table", False):
            self._show_node_detail()

    def action_drill_down(self):
        self._show_node_detail()

    def _show_node_detail(self):
        """Show detailed per-node status for the currently selected robot."""
        node_table = self.query_one("#node_summary_table", DataTable)
        detail = self.query_one("#node_detail_view", Static)

        try:
            from textual.coordinate import Coordinate
            row_idx = node_table.cursor_coordinate.row
            robot_name = str(node_table.get_cell_at(Coordinate(row_idx, 0)))
        except Exception:
            return

        nodes = self._node_status.get(robot_name)
        if not nodes:
            detail.update(f"[dim]No node data for {robot_name}.[/]")
            return

        t = Table(box=None, expand=True, pad_edge=False, title=f"Node details: {robot_name}")
        t.add_column("Status", no_wrap=True, min_width=8)
        t.add_column("Node", no_wrap=True, min_width=12)
        t.add_column("Notes", no_wrap=False)
        for n in sorted(nodes, key=lambda x: x["nickname"]):
            status = n["status"]
            color = _STATUS_COLORS.get(status, "white")
            sname = _STATUS_NAMES.get(status, "?")
            notes = n.get("notes", "")
            t.add_row(f"[{color}]{sname}[/{color}]", n["nickname"], notes)
        detail.update(t)

    def action_stop_save(self):
        """Send shutdown signal to all online robots via tmux."""
        log = self.query_one("#monitor_log", RichLog)
        online_machines = [m for m in self.ctx.runtime_config.values() if m.get("online", False)]
        if not online_machines:
            log.write("[red]No online machines to stop.[/]")
            return

        names = ", ".join(m["name"] for m in sorted(online_machines, key=lambda x: x["name"]))
        log.write(f"[yellow]Sending Stop & Save to: {names}[/]")

        def do_stop():
            def stop_one(m):
                ok, msg = send_tmux_keys(m["user"], m["ip"])
                return m["name"], ok, msg
            return run_parallel(stop_one, online_machines)

        def on_done(results):
            for item, result in sorted(results, key=lambda x: x[0]["name"]):
                if isinstance(result, Exception):
                    log.write(f"  [red]{item['name']}: {result}[/]")
                else:
                    mname, ok, msg = result
                    color = "green" if ok else "red"
                    log.write(f"  [{color}]{mname}: {msg}[/{color}]")
            log.write("[bold]Stop & Save complete.[/]")

        def bg():
            results = do_stop()
            self.app.call_from_thread(on_done, results)

        threading.Thread(target=bg, daemon=True).start()

    def action_gen_rviz(self):
        """Generate a namespaced rviz config for a robot."""
        log = self.query_one("#monitor_log", RichLog)
        # Use the first robot in self.ctx.runtime_config as default
        robots = [m for m in sorted(self.ctx.runtime_config) if self.ctx.runtime_config[m].get("role") == "robot"]
        if not robots:
            log.write("[red]No robots in config.[/]")
            return

        # Generate for each robot
        for robot_name in robots:
            mconf = self.ctx.runtime_config[robot_name]
            platform_id = mconf.get("platform_id", robot_name)
            out_path = generate_namespaced_rviz(robot_name)
            log.write(f"  Generated [cyan]{out_path}[/] for {robot_name}")

        log.write(f"[bold]RViz configs generated.[/] Launch with:")
        log.write(f"  [dim]rviz2 -d /tmp/dcist_<robot>.rviz[/]")

    def action_check_zenoh(self):
        """Check zenoh port reachability + Silvus radio link quality for all machines."""
        log = self.query_one("#monitor_log", RichLog)
        log.write("[dim]Checking zenoh connectivity and radio link quality...[/]")

        zenoh_conf = self.ctx.topo.get("zenoh", {})
        zenoh_port = zenoh_conf.get("port", 7447)

        zenoh_admin_port = zenoh_conf.get("admin_port", 8000)

        def do_check():
            machines = list(self.ctx.runtime_config.values())
            sys_radios = self.ctx.topo.get("silvus_radios", {}).get("radios", {})
            silvus_mgmt_ips = [r["mgmt_ip"] for r in sys_radios.values() if "mgmt_ip" in r]

            results = []
            for m in machines:
                ip = m["ip"]
                zenoh_ok = check_zenoh_port(ip, port=zenoh_port)

                radio = None
                if m.get("online") and silvus_mgmt_ips:
                    try:
                        radio = get_silvus_link_quality(m["user"], ip, silvus_mgmt_ips)
                    except Exception:
                        pass

                results.append((m, zenoh_ok, radio))

            # Query Zenoh admin API from background thread
            import urllib.request as _urlreq
            zenoh_peers = None
            zenoh_peers_err = None
            try:
                url = f"http://localhost:{zenoh_admin_port}/@/router/local/link/**"
                with _urlreq.urlopen(url, timeout=2) as resp:
                    zenoh_peers = json.loads(resp.read())
            except Exception as e:
                zenoh_peers_err = str(e)

            return results, zenoh_peers, zenoh_peers_err

        def on_done(results, zenoh_peers, zenoh_peers_err):
            log.write(f"\n[bold]Zenoh Port {zenoh_port} Reachability:[/]")
            for m, zenoh_ok, _ in sorted(results, key=lambda x: x[0]["name"]):
                color = "green" if zenoh_ok else "red"
                status = "OPEN" if zenoh_ok else "CLOSED"
                log.write(f"  [{color}]{m['name']:12s} {m['ip']:18s} {status}[/{color}]")

            has_radio = any(r for _, _, r in results if r)
            if has_radio:
                log.write(f"\n[bold]Silvus Radio Link Quality:[/]")
                radio_assignment = []  # (machine_name, node_id)
                for m, _, radio in sorted(results, key=lambda x: x[0]["name"]):
                    if not radio:
                        continue
                    if "error" in radio:
                        log.write(f"  [red]{m['name']:12s} {radio['error']}[/]")
                    elif "raw" in radio:
                        log.write(f"  [dim]{m['name']:12s} {radio['raw']}[/]")
                    else:
                        bat = radio.get("battery", "?")
                        mesh = radio.get("mesh_nodes", "?")
                        neighbors = radio.get("neighbor_count", "?")
                        rssi = radio.get("rssi", "N/A")
                        snr = radio.get("snr", "N/A")
                        log.write(
                            f"  [cyan]{m['name']:12s}[/] "
                            f"Bat:{bat}% Mesh:{mesh} Neighbors:{neighbors} "
                            f"RSSI:{rssi} SNR:{snr}"
                        )
                        nid = radio.get("node_id")
                        if nid:
                            radio_assignment.append((m["name"], nid))

                if radio_assignment:
                    log.write(f"\n[bold]Radio Assignment (current):[/]")
                    for mname, nid in radio_assignment:
                        log.write(f"  [cyan]{mname:12s}[/] → node {nid}")
                    log.write(f"  [dim]Press F4 to see RSSI matrix and mesh topology.[/]")

            # ---- Zenoh peer browser ----
            log.write(f"\n[bold]Zenoh Connected Peers:[/]")
            if zenoh_peers is not None:
                if zenoh_peers:
                    for zid, meta in zenoh_peers.items():
                        whatami = meta.get("whatami", "?") if isinstance(meta, dict) else "?"
                        short_zid = zid[:8] if len(zid) > 8 else zid
                        log.write(f"  [green]{short_zid}[/] ({whatami})")
                else:
                    log.write("  [dim]No peers connected.[/]")
            else:
                err_str = zenoh_peers_err or ""
                if "refused" in err_str or "timed out" in err_str or "unreachable" in err_str:
                    log.write(f"  [dim]Admin API not available — start zenohd with --rest-http-port {zenoh_admin_port}[/]")
                else:
                    log.write(f"  [dim]Admin API error: {err_str[:60]}[/]")

            log.write("[bold]Zenoh/radio check complete.[/]")

        def bg():
            results, zenoh_peers, zenoh_peers_err = do_check()
            self.app.call_from_thread(on_done, results, zenoh_peers, zenoh_peers_err)

        threading.Thread(target=bg, daemon=True).start()

    def action_check_silvus_routes(self):
        """Check if Silvus management route exists on all machines + locally."""
        log = self.query_one("#monitor_log", RichLog)
        log.write("[dim]Checking Silvus management routes...[/]")

        mgmt_subnet = self.ctx.topo.get("silvus_radios", {}).get("mgmt_subnet", "172.20.0.0/16")

        def do_check():
            results = []
            # Check local machine first
            local_info = check_silvus_route(mgmt_subnet=mgmt_subnet)
            results.append(("LOCAL", None, local_info))

            # Check all online machines
            machines = [m for m in self.ctx.runtime_config.values() if m.get("online")]
            for m in machines:
                try:
                    info = check_silvus_route(m["user"], m["ip"], mgmt_subnet)
                    results.append((m["name"], m, info))
                except Exception as e:
                    results.append((m["name"], m, {"has_route": False, "interface": None, "silvus_iface": None, "error": str(e)}))
            return results

        def on_done(results):
            log.write(f"\n[bold]Silvus Management Route ({mgmt_subnet}):[/]")
            missing = []
            for name, m, info in sorted(results, key=lambda x: x[0]):
                iface = info.get("silvus_iface") or "—"
                if info.get("has_route"):
                    route = info.get("route_info", "OK")
                    log.write(f"  [green]{name:12s}[/] iface={iface}  [green]ROUTE OK[/] ({route})")
                elif info.get("silvus_iface"):
                    log.write(f"  [red]{name:12s}[/] iface={iface}  [red]NO ROUTE[/]")
                    missing.append((name, m, iface))
                else:
                    log.write(f"  [dim]{name:12s}[/] [dim]No Silvus interface (no 192.168.100.x address)[/]")
            if missing:
                log.write(f"\n[bold yellow]Run these commands to add missing routes:[/]")
                for name, m, iface in missing:
                    if m is None:
                        # local machine
                        log.write(f"\n  [bold]{name}[/] (run locally):")
                        log.write(f"    [cyan]sudo ip route add {mgmt_subnet} dev {iface}[/]")
                        log.write(f"  [dim]To make permanent (same USB dongle):[/]")
                        log.write(f"    [cyan]conn=$(nmcli -f NAME,DEVICE con show | grep {iface} | awk '{{print $1}}')[/]")
                        log.write(f"    [cyan]nmcli con modify \"$conn\" +ipv4.routes \"{mgmt_subnet}\"[/]")
                        log.write(f"    [cyan]nmcli con up \"$conn\"[/]")
                    else:
                        user, ip = m["user"], m["ip"]
                        log.write(f"\n  [bold]{name}[/] (SSH to {ip}):")
                        log.write(f"    [cyan]ssh {user}@{ip} sudo ip route add {mgmt_subnet} dev {iface}[/]")
                        log.write(f"  [dim]To make permanent (same USB dongle):[/]")
                        log.write(f"    [cyan]ssh {user}@{ip}[/]")
                        log.write(f"    [cyan]  conn=$(nmcli -f NAME,DEVICE con show | grep {iface} | awk '{{print $1}}')[/]")
                        log.write(f"    [cyan]  nmcli con modify \"$conn\" +ipv4.routes \"{mgmt_subnet}\"[/]")
                        log.write(f"    [cyan]  nmcli con up \"$conn\"[/]")

        def bg():
            results = do_check()
            self.app.call_from_thread(on_done, results)

        threading.Thread(target=bg, daemon=True).start()

    def action_silvus_detail(self):
        """Show Silvus RSSI link quality matrix and mesh routing topology."""
        log = self.query_one("#monitor_log", RichLog)
        sys_radios = self.ctx.topo.get("silvus_radios", {}).get("radios", {})
        if not sys_radios:
            log.write("[yellow]No Silvus radios defined in topology.[/]")
            return
        log.write("[dim]Querying Silvus radios (direct HTTP)...[/]")

        def do_query():
            return get_silvus_radio_details(self.ctx.topo)

        def on_done(details):
            if not details:
                log.write("[red]No radio data returned.[/]")
                return

            # Build node_id → radio_name reverse map
            nid_to_name = {}
            for rname, info in details.items():
                nid = info.get("node_id")
                if nid:
                    nid_to_name[int(nid)] = rname

            names = sorted(details.keys())
            col_w = max(len(n) for n in names) + 2

            # ---- RSSI Matrix ----
            log.write(f"\n[bold]Silvus RSSI Link Quality:[/]")
            header = " " * (col_w + 2) + "".join(f"{n:>{col_w}}" for n in names)
            log.write(f"  [dim]{header}[/]")

            for src in names:
                info = details[src]
                if info.get("error"):
                    log.write(f"  [red]{src:<{col_w}}[/]  [red](unreachable: {info['error'][:40]})[/]")
                    continue
                row = f"  [cyan]{src:<{col_w}}[/]"
                link_stats = info.get("link_stats")
                rssi_raw = info.get("rssi")
                routing_tree = [int(x) for x in (info.get("routing_tree") or [])]
                src_nid = info.get("node_id")

                for dst in names:
                    if src == dst:
                        row += f"{'---':>{col_w}}"
                        continue

                    dst_nid = details[dst].get("node_id")
                    val = None

                    # Try link_stats (keyed by node_id string)
                    if isinstance(link_stats, dict) and dst_nid is not None:
                        try:
                            val = float(link_stats.get(str(dst_nid)) or link_stats.get(str(int(dst_nid))))
                        except (TypeError, ValueError):
                            pass

                    # Fallback: rssi field (single-neighbor case)
                    if val is None and rssi_raw and isinstance(rssi_raw, list) and len(names) == 2:
                        try:
                            val = float(rssi_raw[0]) / 2.0
                        except (TypeError, ValueError):
                            pass

                    if val is not None:
                        if val > -60:
                            cell = f"[green]{val:.0f}dB[/]"
                        elif val > -80:
                            cell = f"[yellow]{val:.0f}dB[/]"
                        else:
                            cell = f"[red]{val:.0f}dB[/]"
                        row += f"{cell:>{col_w}}"
                    elif dst_nid is not None and int(dst_nid) in routing_tree:
                        row += f"[green]{'linked':>{col_w}}[/]"
                    else:
                        row += f"[dim]{'?':>{col_w}}[/]"
                log.write(row)

            # ---- Mesh Topology ----
            log.write(f"\n[bold]Mesh Routing (per radio):[/]")
            for rname in names:
                info = details[rname]
                if info.get("error"):
                    continue
                nid = info.get("node_id", "?")
                tree = info.get("routing_tree") or []
                # Resolve node IDs to radio names where possible
                tree_str = ", ".join(
                    nid_to_name.get(int(n), str(n)) for n in tree
                ) if tree else "—"
                bat = info.get("battery")
                bat_str = f"  bat={bat:.0f}%" if bat is not None else ""
                log.write(f"  [cyan]{rname}[/] (node {nid}){bat_str}: [{tree_str}]")

            mgmt_subnet = self.ctx.topo.get("silvus_radios", {}).get("mgmt_subnet", "172.20.0.0/16")
            all_err = all(d.get("error") for d in details.values())
            if all_err:
                log.write(f"\n[yellow]All radios unreachable — is the {mgmt_subnet} route set? (press F3)[/]")
            else:
                log.write(f"\n[dim]Press 'z' to see which robot currently has each radio.[/]")

        def bg():
            result = do_query()
            self.app.call_from_thread(on_done, result)

        threading.Thread(target=bg, daemon=True).start()

    def action_start_zenohd(self):
        """Manually start/restart the fleet-managed zenohd."""
        log = self.query_one("#monitor_log", RichLog)

        # Check if a system zenohd is already running (e.g. from base station tmux)
        system_zenohd = subprocess.run(
            ["pgrep", "-f", "rmw_zenohd"], capture_output=True
        ).returncode == 0
        fleet_zenohd = _fleet_zenoh["proc"] and _fleet_zenoh["proc"].poll() is None

        if system_zenohd and not fleet_zenohd:
            log.write("[green]Base station zenohd is running on port 7447 — using it for ROS monitoring.[/]")
            log.write("[dim]Press 'n' to refresh node status.[/]")
            return
        elif fleet_zenohd:
            log.write("[dim]Restarting fleet zenohd...[/]")
            self._stop_zenohd and self._stop_zenohd()

        ok, msg = (self._start_zenohd() if self._start_zenohd else (False, "start_zenohd not configured"))
        color = "green" if ok else "red"
        log.write(f"[{color}]Fleet zenohd: {msg}[/{color}]")
        if ok:
            log.write("[dim]Press 'n' to refresh node status.[/]")

    def action_bandwidth_test(self):
        """Run iperf3 throughput test from base station to selected online robots."""
        import shutil
        log = self.query_one("#monitor_log", RichLog)

        machines = [m for m in self.ctx.runtime_config.values()
                    if m.get("online") and m.get("role") == "robot"]
        if not machines:
            log.write("[yellow]No online robots to test.[/]")
            return

        if not shutil.which("iperf3"):
            log.write("[red]iperf3 not found locally — run: sudo apt install iperf3[/]")
            return

        server_ip = get_local_ip_for_network(self.ctx.topo, self.ctx.active_network)
        if not server_ip:
            log.write(f"[red]Cannot determine local IP on network '{self.ctx.active_network}'.[/]")
            return

        def on_results(results):
            if not results:
                return
            log.write(f"[dim]── bandwidth test summary ({self.ctx.active_network}) ──[/]")
            for r in results:
                name = r.get("name", "?")
                if "error" in r:
                    log.write(f"[red]{name}: {r['error']}[/]")
                else:
                    mbps = r.get("mbps", 0)
                    color = "green" if mbps > 10 else "yellow" if mbps > 1 else "red"
                    tunnel_warn = "  [yellow]SSH tunnel (direct port blocked)[/]" if r.get("tunneled") else ""
                    log.write(
                        f"[{color}]{name}: {mbps:.1f} Mbps[/]"
                        f"  retransmits={r.get('retransmits', 0)}"
                        f"{tunnel_warn}"
                    )

        self.app.push_screen(BandwidthSelectScreen(machines, server_ip), on_results)

# ---- Zenoh Screen ----

