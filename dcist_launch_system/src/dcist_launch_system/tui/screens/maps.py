"""Maps browser screen + PriorMapSelector widget."""
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

class PriorMapSelector(Vertical):
    """Reusable widget for browsing and selecting prior maps from fleet nodes.

    Uses the same Tree-based approach as MapsScreen. Click an experiment
    to select it. Experiments with both hydra + roman are marked with a green check.
    """
    def __init__(self, ctx: TuiContext, **kwargs):
        super().__init__(**kwargs)
        self.ctx = ctx

    def compose(self) -> ComposeResult:
        yield Label("Prior map (click to select, or type custom path):")
        yield Input(placeholder="Filter experiments...", id="prior_search")
        yield Tree("Fleet Experiments", id="prior_tree")
        yield Input(placeholder="(optional) custom: machine:~/adt4_output/exp", id="prior_input")

    def on_mount(self):
        self._selected_path = ""
        self._selected_source = None
        self._all_results = []
        self.set_timer(0.5, self._safe_refresh)

    def _safe_refresh(self):
        """Guard against NoActiveAppError on re-mount of cached screens."""
        try:
            self.app  # test if connected
            self.refresh_priors()
        except Exception:
            pass

    def refresh_priors(self, log_widget=None):

        def do_fetch():
            machines = [m for m in self.ctx.runtime_config.values() if m.get("online", False)]
            def fetch(m):
                return list_remote_experiments(m["user"], m["ip"], self.ctx.output_root)
            return run_parallel(fetch, machines)

        def on_done(results):
            self._all_results = results
            try:
                self._populate_tree()
            except Exception:
                pass
            if log_widget:
                total = sum(
                    len(exps) for _, exps in results
                    if not isinstance(exps, Exception)
                )
                log_widget.write(f"[green]Found {total} experiments across fleet.[/]")

        def _bg():
            result = do_fetch()
            try:
                self.app.call_from_thread(on_done, result)
            except Exception:
                pass

        threading.Thread(target=_bg, daemon=True).start()

    def _populate_tree(self, query=""):
        tree = self.query_one("#prior_tree", Tree)
        tree.clear()
        for m, exps in sorted(self._all_results, key=lambda x: x[0]["name"]):
            if isinstance(exps, Exception):
                node = tree.root.add(m["name"], expand=True)
                node.add_leaf(f"[red]Error: {exps}[/]")
                continue
            filtered = [
                e for e in sorted(exps, key=lambda e: e["name"])
                if self.ctx.fuzzy_match(query, e["name"])
            ] if query else sorted(exps, key=lambda e: e["name"])
            if not filtered and query:
                continue
            node = tree.root.add(m["name"], expand=True)
            if not filtered:
                node.add_leaf("[dim]no experiments[/]")
                continue
            for exp in filtered:
                subdirs = exp.get("subdirs", [])
                has_prior = "hydra" in subdirs and "roman" in subdirs
                tag = " [green]\u2714[/]" if has_prior else ""
                label = f"{exp['name']}  ({exp.get('size', '?')}){tag}"
                leaf = node.add_leaf(label)
                leaf.data = {
                    "machine": m["name"],
                    "experiment": exp["name"],
                    "path": f"{self.ctx.output_root}/{exp['name']}",
                }
        tree.root.expand_all()

    def on_input_changed(self, event: Input.Changed):
        if event.input.id == "prior_search" and self._all_results:
            self._populate_tree(event.value.strip())

    def on_tree_node_selected(self, event):
        node = event.node
        if hasattr(node, "data") and node.data and "experiment" in node.data:
            self._selected_path = node.data["path"]
            self._selected_source = node.data["machine"]

    def get_prior_map(self):
        """Return (prior_path, source_machine_name) or ("", None)."""
        custom = self.query_one("#prior_input", Input).value.strip()
        if custom:
            if ":" in custom:
                parts = custom.split(":", 1)
                return parts[1], parts[0]
            return custom, None
        if self._selected_path:
            return self._selected_path, self._selected_source
        return "", None



class MapsScreen(ModalScreen):
    BINDINGS = [
        Binding("escape", "dismiss", "Back", priority=True),
        Binding("t", "transfer_selected", "Transfer", priority=True),
        Binding("v", "verify_selected", "Verify", priority=True),
        Binding("r", "refresh_maps", "Refresh", priority=True),
    ]

    def __init__(self, ctx: TuiContext):
        super().__init__()
        self.ctx = ctx
        self.selected_machine = None
        self.selected_experiment = None
        self._all_results = []  # cached fetch results

    def compose(self) -> ComposeResult:
        yield Vertical(
            Label("[bold]Map Browser[/] — select an experiment"),
            Input(placeholder="Search experiments...", id="map_search"),
            Rule(),
            Tree("Fleet Maps", id="map_tree"),
            Rule(),
            RichLog(id="map_log", highlight=True, markup=True, max_lines=50),
        )
        yield Footer()

    def action_refresh_maps(self):
        self._refresh_maps()

    def on_mount(self):
        self._refresh_maps()

    def _refresh_maps(self):
        tree = self.query_one("#map_tree", Tree)
        log = self.query_one("#map_log", RichLog)
        tree.clear()
        log.write("[dim]Fetching experiments from fleet...[/]")

        def do_fetch():
            machines = [m for m in self.ctx.runtime_config.values() if m.get("online", False)]

            def fetch(m):
                return list_remote_experiments(m["user"], m["ip"], self.ctx.output_root)

            return run_parallel(fetch, machines)

        def on_done(results):
            self._all_results = results
            query = self.query_one("#map_search", Input).value.strip()
            self._populate_tree(query)
            log.write("[green]Maps loaded.[/]")

        def bg():
            results = do_fetch()
            self.app.call_from_thread(on_done, results)

        threading.Thread(target=bg, daemon=True).start()

    def _populate_tree(self, query=""):
        tree = self.query_one("#map_tree", Tree)
        tree.clear()
        for m, exps in sorted(self._all_results, key=lambda x: x[0]["name"]):
            if isinstance(exps, Exception):
                node = tree.root.add(m["name"], expand=True)
                node.data = {"machine": m["name"]}
                node.add_leaf(f"[red]Error: {exps}[/]")
                continue
            filtered = [
                e for e in sorted(exps, key=lambda e: e["name"])
                if self.ctx.fuzzy_match(query, e["name"])
            ] if query else sorted(exps, key=lambda e: e["name"])
            if not filtered and query:
                continue  # hide machines with no matches
            node = tree.root.add(m["name"], expand=True)
            node.data = {"machine": m["name"]}
            if not filtered:
                node.add_leaf("[dim]no experiments[/]")
                continue
            for exp in filtered:
                subdirs_str = ", ".join(exp["subdirs"]) if exp["subdirs"] else ""
                label = f"{exp['name']}  [{subdirs_str}]  ({exp['size']})"
                leaf = node.add_leaf(label)
                leaf.data = {"machine": m["name"], "experiment": exp["name"]}
        tree.root.expand_all()

    def on_input_changed(self, event: Input.Changed):
        if event.input.id == "map_search" and self._all_results:
            self._populate_tree(event.value.strip())

    def on_tree_node_selected(self, event):
        node = event.node
        if hasattr(node, "data") and node.data and "experiment" in node.data:
            self.selected_machine = node.data["machine"]
            self.selected_experiment = node.data["experiment"]
            log = self.query_one("#map_log", RichLog)
            log.write(
                f"Selected: [bold]{self.selected_machine}[/]:"
                f"[cyan]{self.selected_experiment}[/]"
                f" — press [bold]T[/] to transfer, [bold]V[/] to verify"
            )

    def action_transfer_selected(self):
        if not self.selected_experiment:
            self.query_one("#map_log", RichLog).write("[yellow]Select an experiment first.[/]")
            return
        self.app.push_screen(
            TransferScreen(self.ctx, self.selected_machine, self.selected_experiment)
        )

    def action_verify_selected(self):
        if not self.selected_experiment:
            self.query_one("#map_log", RichLog).write("[yellow]Select an experiment first.[/]")
            return
        self.app.push_screen(VerifyScreen(self.ctx, self.selected_experiment))

# ---- Transfer Screen ----
