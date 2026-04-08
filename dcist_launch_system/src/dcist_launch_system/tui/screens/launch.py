"""Launch screen — configure and start experiment sessions."""
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

from dcist_launch_system.tui.screens.confirm import ConfirmScreen


def check_sessions_and_prompt(app, targets, session_name, log):
    """Check for running tmux sessions and prompt user in TUI threads to kill them."""
    def check(m):
        rc, _, _ = ssh_cmd(m["user"], m["ip"], f"tmux has-session -t {shlex.quote(session_name)} 2>/dev/null", timeout=5)
        if rc == 0:
            return m["name"]
        return None

    app.call_from_thread(log.write, f"  [dim]Checking for existing '{session_name}' sessions...[/]")
    results = run_parallel(check, targets)
    conflicts = [r for _, r in results if isinstance(r, str)]

    if not conflicts:
        return True

    app.call_from_thread(
        log.write,
        f"  [yellow]Session '{session_name}' already running on: {', '.join(conflicts)}[/]"
    )

    user_decision = [None]
    ev = threading.Event()

    def ask():
        def on_dismiss(result):
            user_decision[0] = result
            ev.set()
        msg = f"Session '{session_name}' is already running on {len(conflicts)} machine(s):\n{', '.join(conflicts)}\n\nTerminate existing sessions and proceed?"
        app.push_screen(ConfirmScreen(msg, yes_label="Yes, Terminate (y)", no_label="No, Abort (n)"), on_dismiss)

    app.call_from_thread(ask)
    ev.wait()

    if user_decision[0]:
        for m in targets:
            if m["name"] in conflicts:
                app.call_from_thread(log.write, f"  [dim]Terminating session {session_name} on {m['name']}...[/]")
                ssh_cmd(m["user"], m["ip"], f"tmux kill-session -t {shlex.quote(session_name)}", timeout=5)
        return True
    else:
        app.call_from_thread(log.write, "[red]Launch aborted by user.[/]")
        return False


class LaunchScreen(ModalScreen):
    BINDINGS = [
        Binding("escape", "dismiss", "Back", priority=True),
        Binding("m", "preset_mapping", "Mapping (m)", priority=True),
        Binding("r", "preset_relocalize", "Relocalize (r)", priority=True),
        Binding("d", "dry_run", "Dry Run", priority=True),
        Binding("g", "do_launch", "Launch (g)", priority=True),
        Binding("z", "deploy_zenoh", "Deploy Zenoh (z)", priority=True),
        Binding("p", "browse_prior", "Refresh Priors (p)", priority=True),
        Binding("f2", "toggle_log_size", "Log Size [F2]", priority=True),
    ]

    DEFAULT_CSS = """
    LaunchScreen #robot_select {
        max-height: 6;
    }
    LaunchScreen #session_select {
        max-height: 12;
    }
    LaunchScreen #bs_select {
        max-height: 4;
    }
    LaunchScreen #bs_session_select {
        max-height: 6;
    }
    LaunchScreen #prior_tree {
        min-height: 8;
        max-height: 16;
    }
    LaunchScreen #launch_log {
        min-height: 12;
    }
    """

    _LOG_SIZES = [12, 24, 40]

    def __init__(self, ctx: TuiContext, stop_zenohd=None, mode=None):
        super().__init__()
        self.ctx = ctx
        self._stop_zenohd = stop_zenohd
        self._launching = False
        self._mode = mode  # None=manual, "mapping", "relocalize"
        self._log_size_idx = 0

    def compose(self) -> ComposeResult:
        robots = [
            (m["name"], m["name"], m.get("online", False))
            for m in self.ctx.runtime_config.values()
            if m["role"] == "robot"
        ]
        base_stations = [
            (m["name"], f"{m['name']} (local)" if m["ip"] in self.ctx._local_ips else m["name"], m.get("online", False) or m["ip"] in self.ctx._local_ips)
            for m in self.ctx.runtime_config.values()
            if m["role"] == "base_station"
        ]
        self._all_sessions = sorted(
            [f.stem for f in _TMUX_DIR.glob("*.yaml")]
        ) if _TMUX_DIR.exists() else []

        yield VerticalScroll(
            Label("[bold]Launch[/] — m=mapping, r=relocalize, g=launch, d=dry run, p=refresh priors"),
            Rule(),
            Label("Select robots:"),
            SelectionList(*robots, id="robot_select"),
            Rule(),
            Label("Robot session:"),
            Input(placeholder="Filter sessions...", id="session_search"),
            SelectionList(*[(s, s) for s in self._all_sessions], id="session_select"),
            Rule(),
            Label("Base station (optional):"),
            SelectionList(*base_stations, id="bs_select"),
            Label("Base station session:"),
            SelectionList(
                *[(s, s) for s in self._all_sessions if "base_station" in s],
                id="bs_session_select",
            ),
            Rule(),
            Label("Experiment (saved to ~/adt4_output/<name>/, e.g. 03252026_building_test):"),
            Input(placeholder="date_description", id="experiment_input"),
            Rule(),
            PriorMapSelector(id="launch_prior_selector"),
            Rule(),
            Vertical(id="launch_progress_area"),
            RichLog(id="launch_log", highlight=True, markup=True, max_lines=100),
        )
        yield Footer()

    def on_mount(self):
        if self._mode:
            self._apply_preset(self._mode, update_robots=True)

    def action_preset_mapping(self):
        self._mode = "mapping"
        # Save to app so re-opening preserves mode
        try:
            self.app._last_launch_mode = "mapping"
        except Exception:
            pass
        self._apply_preset("mapping")

    def action_preset_relocalize(self):
        self._mode = "relocalize"
        try:
            self.app._last_launch_mode = "relocalize"
        except Exception:
            pass
        self._apply_preset("relocalize")

    def _apply_preset(self, mode, update_robots=False):
        """Auto-populate selections from fleet_defaults in topology.

        update_robots: if True, reset robot selection to all online robots
        (only done on initial mount, not when the user switches mode).
        """
        defaults = self.ctx.topo.get("fleet_defaults", {})
        spot_platforms = set(defaults.get("spot_platforms", ["smaug", "topaz"]))
        sessions_cfg = defaults.get("sessions", {})
        mode_sessions = sessions_cfg.get(mode, {})
        spot_session = mode_sessions.get("spot", "")
        phoenix_session = mode_sessions.get("phoenix", "")
        bs_session = defaults.get("base_station_session", "")

        log = self.query_one("#launch_log", RichLog)
        log.write(f"[bold cyan]Auto-populating for {mode} mode...[/]")

        # Select online robots (only on initial mount)
        robot_sel = self.query_one("#robot_select", SelectionList)
        if update_robots:
            robot_sel.deselect_all()
            for m in self.ctx.runtime_config.values():
                if m["role"] == "robot" and m.get("online"):
                    robot_sel.select(m["name"])

        # Pick session based on first selected (or online) robot's platform
        selected_robots = list(robot_sel.selected)
        first_robot = next(
            (self.ctx.runtime_config[n] for n in selected_robots if n in self.ctx.runtime_config),
            None,
        ) or next(
            (m for m in self.ctx.runtime_config.values()
             if m["role"] == "robot" and m.get("online")),
            None,
        )
        target_session = ""
        if first_robot:
            target_session = spot_session if first_robot.get("platform_id") in spot_platforms else phoenix_session

        if target_session:
            sess_sel = self.query_one("#session_select", SelectionList)
            sess_sel.deselect_all()
            sess_sel.select(target_session)
            log.write(f"  Robot session: [cyan]{target_session}[/]")
            # Scroll to selected item
            try:
                idx = self._all_sessions.index(target_session)
                sess_sel.scroll_to_center(sess_sel.get_option_at_index(idx))
            except Exception:
                pass

        # Select local base station
        local_bs = None
        bs_sel = self.query_one("#bs_select", SelectionList)
        bs_sel.deselect_all()
        for m in self.ctx.runtime_config.values():
            if m["role"] == "base_station" and m["ip"] in self.ctx._local_ips:
                bs_sel.select(m["name"])
                local_bs = m["name"]
                break
        if local_bs:
            log.write(f"  Base station: [cyan]{local_bs} (local)[/]")

        # Select BS session
        bs_sess_sel = self.query_one("#bs_session_select", SelectionList)
        bs_sess_sel.deselect_all()
        if bs_session:
            bs_sess_sel.select(bs_session)
            log.write(f"  BS session: [cyan]{bs_session}[/]")

        if mode == "relocalize":
            log.write("[yellow]Relocalize mode: prior map is required. Select one below or press p to refresh.[/]")
        log.write("[dim]Review selections, then g=launch, d=dry run.[/]")

    def on_input_changed(self, event: Input.Changed):
        if event.input.id == "session_search":
            query = event.value.strip()
            sel = self.query_one("#session_select", SelectionList)
            sel.clear_options()
            for s in self._all_sessions:
                if self.ctx.fuzzy_match(query, s):
                    sel.add_option((s, s))

    def _get_selected_session(self):
        sel = self.query_one("#session_select", SelectionList)
        selected = list(sel.selected)
        return selected[0] if selected else ""

    def action_browse_prior(self):
        log = self.query_one("#launch_log", RichLog)
        self.query_one("#launch_prior_selector", PriorMapSelector).refresh_priors(log)

    def action_deploy_zenoh(self):
        """Deploy zenoh config to base station (robots just listen)."""
        log = self.query_one("#launch_log", RichLog)

        # Get selected robots (for endpoint scoping)
        robot_sel = self.query_one("#robot_select", SelectionList)
        robot_names = list(robot_sel.selected)

        # Only deploy to base station
        bs_sel = self.query_one("#bs_select", SelectionList)
        bs_names = list(bs_sel.selected)
        if not bs_names or bs_names[0] not in self.ctx.runtime_config:
            log.write("[yellow]No base station selected.[/]")
            return

        targets = [self.ctx.runtime_config[bs_names[0]]]
        log.write(f"[bold]Deploying zenoh config to {targets[0]['name']}...[/]")
        log.write(f"  [dim]Endpoints → robots: {', '.join(robot_names)}[/]")

        def do_deploy():
            results = []
            for m in targets:
                try:
                    endpoints = generate_zenoh_endpoints(
                        self.ctx.topo, self.ctx.active_network, m["name"],
                        robot_filter=robot_names,
                    )
                    ok, msg = deploy_zenoh_config(m["user"], m["ip"], endpoints)
                    results.append((m["name"], ok, msg, len(endpoints)))
                except KeyError:
                    results.append((m["name"], False, f"no address on {self.ctx.active_network}", 0))
            return results

        def on_done(results):
            for name, ok, msg, n_ep in results:
                color = "green" if ok else "red"
                log.write(f"  [{color}]{name}: {msg} ({n_ep} endpoint(s))[/{color}]")
            log.write("[bold]Zenoh deploy complete.[/]")

        def bg():
            results = do_deploy()
            self.app.call_from_thread(on_done, results)

        threading.Thread(target=bg, daemon=True).start()

    def action_toggle_log_size(self):
        self._log_size_idx = (self._log_size_idx + 1) % len(self._LOG_SIZES)
        new_h = self._LOG_SIZES[self._log_size_idx]
        log = self.query_one("#launch_log", RichLog)
        log.styles.min_height = new_h

    def _build_commands(self):
        log = self.query_one("#launch_log", RichLog)
        robot_sel = self.query_one("#robot_select", SelectionList)
        selected = list(robot_sel.selected)
        if not selected:
            return [], None, "No robots selected."

        session = self._get_selected_session()
        if not session:
            return [], None, "No session selected."

        experiment = self.query_one("#experiment_input", Input).value.strip()
        if not experiment:
            return [], None, "No experiment name provided."

        prior_path, prior_source = self.query_one("#launch_prior_selector", PriorMapSelector).get_prior_map()

        if self._mode == "relocalize" and not prior_path:
            return [], None, "Relocalize mode requires a prior map. Select one or press p to refresh."

        # If prior map comes from a source machine, targets get it as prior_<name>
        if prior_path and prior_source:
            exp_name = pathlib.Path(prior_path).name
            dest_prior = f"{self.ctx.output_root}/prior_{exp_name}"
            # Source machine uses original path, others use transferred path
            prior_paths = {}
            for rname in selected:
                if rname == prior_source:
                    prior_paths[rname] = prior_path
                else:
                    prior_paths[rname] = dest_prior
        else:
            prior_paths = {rname: prior_path for rname in selected}

        cmds = []
        for rname in selected:
            m = self.ctx.runtime_config[rname]
            out_dir = f"{self.ctx.output_root}/{experiment}"
            rprior = prior_paths.get(rname, "")
            prior_flag = f" -p {_quote_path(rprior)}" if rprior else ""
            remote_cmd = (
                f"cd ~/dcist_ws && source install/setup.zsh && "
                f"src/awesome_dcist_t4/dcist_launch_system/bin/run-adt4 "
                f"{shlex.quote(session)} -n {shlex.quote(m['name'])} "
                f"-c {shlex.quote(m['platform_id'])} "
                f"-o {_quote_path(out_dir)} -y{prior_flag} "
                f"--tmuxp-args \"-d\""
            )
            cmds.append((m, remote_cmd))

        prior_info = (prior_path, prior_source) if prior_path else None
        return cmds, prior_info, None


    def action_dry_run(self):
        log = self.query_one("#launch_log", RichLog)
        cmds, prior_info, err = self._build_commands()
        if err:
            log.write(f"[yellow]{err}[/]")
            return
        log.write("[bold cyan]Dry run:[/]")
        if prior_info:
            prior_path, prior_source = prior_info
            if prior_source:
                exp_name = pathlib.Path(prior_path).name
                dest_name = f"prior_{exp_name}"
                for m, cmd in cmds:
                    if m["name"] != prior_source:
                        log.write(
                            f"  [dim]Transfer prior map: {prior_source} -> {m['name']} "
                            f"({exp_name} as {dest_name})[/]"
                        )
                log.write(f"  [dim]Prior map path on targets: {self.ctx.output_root}/{dest_name}[/]")
        for m, cmd in cmds:
            log.write(f"  ssh {m['user']}@{m['ip']} \"{cmd}\"")

    def _transfer_prior_to_robot(self, prior_path, prior_source, target_machine, log, progress_callback=None):
        """Transfer prior map from source machine to target if it doesn't exist."""
        exp_name = pathlib.Path(prior_path).name
        dest_name = f"prior_{exp_name}"
        dest_path = f"{self.ctx.output_root}/{dest_name}"

        # Check if prior already exists on target
        rc, out, _ = ssh_cmd(
            target_machine["user"], target_machine["ip"],
            f"test -d {_quote_path(dest_path + '/hydra')} && test -d {_quote_path(dest_path + '/roman')} && echo OK",
            timeout=10,
        )
        if "OK" in out:
            self.app.call_from_thread(
                log.write,
                f"  [dim]{target_machine['name']}: prior map already present ({dest_name})[/]"
            )
            if progress_callback:
                progress_callback(100)
            return True, dest_path

        # Need to transfer
        self.app.call_from_thread(
            log.write,
            f"  [cyan]Transferring prior map to {target_machine['name']}...[/]"
        )
        src = self.ctx.runtime_config.get(prior_source)
        if not src:
            self.app.call_from_thread(
                log.write,
                f"  [red]Source machine '{prior_source}' not found in config.[/]"
            )
            return False, ""

        ok, method, msg = rsync_transfer(
            src["user"], src["ip"], prior_path,
            target_machine["user"], target_machine["ip"], dest_path,
            relay=False, exclude=["recorded_data/"],
            progress_callback=progress_callback,
        )
        if ok:
            if progress_callback:
                progress_callback(100)
            self.app.call_from_thread(
                log.write,
                f"  [green]{target_machine['name']}: prior map transferred ({method})[/]"
            )
            return True, dest_path
        else:
            self.app.call_from_thread(
                log.write,
                f"  [red]{target_machine['name']}: prior map transfer FAILED: {msg}[/]"
            )
            return False, ""

    def _update_launch_progress(self, name, pct):
        try:
            bar = self.query_one(f"#lpb_{name}", ProgressBar)
            bar.update(progress=pct)
        except Exception:
            pass

    def action_do_launch(self):
        if self._launching:
            return
        log = self.query_one("#launch_log", RichLog)
        cmds, prior_info, err = self._build_commands()
        if err:
            log.write(f"[yellow]{err}[/]")
            return

        self._launching = True
        experiment = self.query_one("#experiment_input", Input).value.strip()
        # Snapshot session name before thread starts (DOM queries from threads are unsafe)
        _launch_session_name = self._get_selected_session()

        # Snapshot base station selection
        robot_sel = self.query_one("#robot_select", SelectionList)
        _robot_names = list(robot_sel.selected)

        bs_sel = self.query_one("#bs_select", SelectionList)
        bs_selected = list(bs_sel.selected)
        bs_sess_sel = self.query_one("#bs_session_select", SelectionList)
        bs_sess_selected = list(bs_sess_sel.selected)

        # Debug: show raw selection values
        log.write(f"[dim]robots.selected = {_robot_names}[/]")
        log.write(f"[dim]session = {_launch_session_name}[/]")
        log.write(f"[dim]bs.selected = {bs_selected}[/]")
        log.write(f"[dim]bs_session.selected = {bs_sess_selected}[/]")
        log.write(f"[dim]bs option_count = {bs_sel.option_count}[/]")

        _bs_machine = self.ctx.runtime_config.get(bs_selected[0]) if bs_selected else None
        _bs_session = bs_sess_selected[0] if bs_sess_selected else None

        # Set up progress bars for transfers if needed
        area = self.query_one("#launch_progress_area", Vertical)
        area.remove_children()
        if prior_info:
            prior_path, prior_source = prior_info
            if prior_source:
                targets = [m for m, _ in cmds if m["name"] != prior_source]
                if targets:
                    area.mount(Label("[bold]Prior map transfer progress:[/]"))
                    for m in targets:
                        area.mount(
                            Vertical(
                                Label(f"  {m['name']}:"),
                                ProgressBar(total=100, show_eta=False, id=f"lpb_{m['name']}"),
                            )
                        )

        def do_launch():
            try:
                # Phase 0: Deploy zenoh config to base station only
                # (robots just listen — star topology, no config change needed)
                deploy_targets = [_bs_machine] if _bs_machine else []
                if deploy_targets:
                    self.app.call_from_thread(
                        log.write, f"[bold]Deploying zenoh config to base station...[/]"
                    )
                    all_ok = True
                    for m in deploy_targets:
                        try:
                            endpoints = generate_zenoh_endpoints(
                                self.ctx.topo, self.ctx.active_network, m["name"],
                                robot_filter=_robot_names,
                            )
                            ok, msg = deploy_zenoh_config(m["user"], m["ip"], endpoints)
                            color = "green" if ok else "red"
                            self.app.call_from_thread(
                                log.write, f"  [{color}]{m['name']}: {msg} ({len(endpoints)} ep)[/{color}]"
                            )
                            if not ok:
                                all_ok = False
                        except KeyError:
                            self.app.call_from_thread(
                                log.write, f"  [dim]{m['name']}: no address on {self.ctx.active_network}[/]"
                            )
                    if not all_ok:
                        self.app.call_from_thread(
                            log.write, "[bold red]ABORT: zenoh deploy failed on some machines.[/]"
                        )
                        return

                # Phase 1: Transfer prior maps if needed
                if prior_info:
                    prior_path, prior_source = prior_info
                    if prior_source:
                        self.app.call_from_thread(
                            log.write, "[bold]Phase 1: Ensuring prior maps on all robots...[/]"
                        )
                        for m, cmd in cmds:
                            if m["name"] == prior_source:
                                continue  # source already has it

                            def make_cb(mname):
                                def cb(pct):
                                    self.app.call_from_thread(
                                        self._update_launch_progress, mname, pct
                                    )
                                return cb

                            ok, dest_path = self._transfer_prior_to_robot(
                                prior_path, prior_source, m, log,
                                progress_callback=make_cb(m["name"]),
                            )
                            if not ok:
                                self.app.call_from_thread(
                                    log.write,
                                    f"  [bold red]ABORT: could not transfer prior to {m['name']}[/]"
                                )
                                return

                # Phase 1.5: Verify no conflicting tmux sessions
                all_targets = [m for m, _ in cmds]
                if _launch_session_name:
                    if not check_sessions_and_prompt(self.app, all_targets, _launch_session_name, log):
                        return

                # Phase 2: Check for existing output dirs
                out_dir = f"{self.ctx.output_root}/{experiment}"
                conflicts = []
                for m, cmd in cmds:
                    rc, out, _ = ssh_cmd(m["user"], m["ip"], f"test -d {_quote_path(out_dir)} && echo EXISTS", timeout=5)
                    if "EXISTS" in out:
                        conflicts.append(m["name"])

                if conflicts:
                    names = ", ".join(conflicts)
                    self.app.call_from_thread(
                        log.write,
                        f"[yellow]{out_dir} already exists on: {names}[/]"
                    )
                    # Prompt user via modal
                    user_decision = [None]
                    ev = threading.Event()

                    def ask_overwrite():
                        def on_dismiss(result):
                            user_decision[0] = result
                            ev.set()
                        msg = (
                            f"Output directory '{experiment}' already exists on:\n"
                            f"{names}\n\n"
                            f"Overwrite existing directories and launch?"
                        )
                        self.app.push_screen(
                            ConfirmScreen(msg, yes_label="Yes, Overwrite (y)", no_label="No, Keep (n)"), on_dismiss
                        )

                    self.app.call_from_thread(ask_overwrite)
                    ev.wait(timeout=120)

                    if user_decision[0] is None:
                        self.app.call_from_thread(
                            log.write, "[red]Launch aborted — confirmation timed out or app closed.[/]"
                        )
                        return
                    if not user_decision[0]:
                        self.app.call_from_thread(
                            log.write, "[red]Launch aborted — existing directories preserved.[/]"
                        )
                        return

                    # Remove existing dirs on conflicting machines
                    for m, cmd in cmds:
                        if m["name"] in conflicts:
                            self.app.call_from_thread(
                                log.write, f"  [dim]Removing {out_dir} on {m['name']}...[/]"
                            )
                            ssh_cmd(m["user"], m["ip"], f"rm -rf {_quote_path(out_dir)}", timeout=15)

                # Launch on each robot
                self.app.call_from_thread(
                    log.write,
                    "[bold]" + ("Phase 2: " if prior_info else "") + "Launching...[/]"
                )
                for m, cmd in cmds:
                    self.app.call_from_thread(
                        log.write, f"  Launching on [bold]{m['name']}[/]..."
                    )
                    rc, out, launch_err = ssh_cmd(m["user"], m["ip"], cmd, timeout=30)
                    if rc == 0:
                        self.app.call_from_thread(
                            log.write, f"  [green]{m['name']}: launched[/]"
                        )
                    else:
                        self.app.call_from_thread(
                            log.write, f"  [red]{m['name']}: FAILED (rc={rc}) {launch_err}[/]"
                        )
                # Phase 3: Launch base station if selected
                if _bs_machine and _bs_session:
                    self.app.call_from_thread(
                        log.write, "[bold]Launching base station...[/]"
                    )
                    # Stop fleet zenohd so BS can bind port 7447
                    if _fleet_zenoh["proc"]:
                        self.app.call_from_thread(
                            log.write, "  [dim]Stopping fleet zenohd (base station will take over)...[/]"
                        )
                        self._stop_zenohd and self._stop_zenohd()

                    # Generate namespaced rviz config:
                    # mapping → first selected robot; relocalize → base station
                    # (BS publishes the prior map in relocalize mode)
                    rviz_flag = ""
                    try:
                        if self._mode == "relocalize":
                            rviz_name = _bs_machine["name"]
                        elif _robot_names:
                            rviz_name = _robot_names[0]
                        else:
                            rviz_name = None
                        if rviz_name:
                            rviz_path = generate_namespaced_rviz(rviz_name)
                            rviz_flag = f" --rviz-config {shlex.quote(rviz_path)}"
                            self.app.call_from_thread(
                                log.write, f"  [dim]RViz config: {rviz_path}[/]"
                            )
                    except Exception as e:
                        self.app.call_from_thread(
                            log.write, f"  [yellow]RViz config generation failed: {e}[/]"
                        )

                    out_dir = f"{self.ctx.output_root}/{experiment}"
                    q_output = _quote_path(out_dir)
                    platform = _bs_machine.get("platform_id") or _bs_machine["name"]
                    config_flag = f" -c {shlex.quote(platform)}" if _bs_machine.get("platform_id") else ""
                    bs_cmd = (
                        f"cd ~/dcist_ws && source install/setup.zsh && "
                        f"nohup src/awesome_dcist_t4/dcist_launch_system/bin/run-adt4 "
                        f"{shlex.quote(_bs_session)} -n {shlex.quote(_bs_machine['name'])}"
                        f"{config_flag} -o {q_output} -y{rviz_flag} "
                        f"--tmuxp-args \"-d\" "
                        f">/dev/null 2>&1 &"
                    )
                    self.app.call_from_thread(
                        log.write, f"  [dim][{_bs_machine['user']}@{_bs_machine['ip']}] {bs_cmd}[/]"
                    )
                    rc, out, err = ssh_cmd(_bs_machine["user"], _bs_machine["ip"], bs_cmd, timeout=60)
                    # nohup & always returns 0; verify tmux session started
                    import time
                    time.sleep(3)
                    tmux_name = "adt4_system"
                    trc, tout, _ = ssh_cmd(
                        _bs_machine["user"], _bs_machine["ip"],
                        f"tmux has-session -t {shlex.quote(tmux_name)} 2>/dev/null && echo OK",
                        timeout=5,
                    )
                    if "OK" in tout:
                        self.app.call_from_thread(
                            log.write, f"  [green]{_bs_machine['name']}: base station launched (tmux session: {tmux_name})[/]"
                        )
                        self.app.call_from_thread(
                            log.write, "  [dim]Base station zenohd is now active on port 7447.[/]"
                        )
                    else:
                        self.app.call_from_thread(
                            log.write, f"  [red]{_bs_machine['name']}: tmux session '{tmux_name}' not found after launch. rc={rc} {err}[/]"
                        )

                self.app.call_from_thread(log.write, "[bold]Launch sequence complete.[/]")
            finally:
                self._launching = False

        threading.Thread(target=do_launch, daemon=True).start()

# ---- Bandwidth Selection Screen ----

