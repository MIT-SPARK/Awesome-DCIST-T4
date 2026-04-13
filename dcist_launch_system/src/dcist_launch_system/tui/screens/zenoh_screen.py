"""Zenoh config management screen."""

from __future__ import annotations

import threading

from textual.app import ComposeResult
from textual.binding import Binding
from textual.containers import Vertical
from textual.screen import ModalScreen
from textual.widgets import (
    DataTable,
    Footer,
    Label,
    RichLog,
    Rule,
)

from dcist_launch_system.fleet_helpers import (
    deploy_zenoh_config,
    generate_zenoh_endpoints,
    run_parallel,
)
from dcist_launch_system.tui.context import TuiContext


class ZenohScreen(ModalScreen):
    BINDINGS = [
        Binding("escape", "dismiss", "Back", priority=True),
        Binding("d", "deploy_all", "Deploy All", priority=True),
        Binding("p", "preview", "Preview", priority=True),
    ]

    def __init__(self, ctx: TuiContext):
        super().__init__()
        self.ctx = ctx

    def compose(self) -> ComposeResult:
        yield Vertical(
            Label("[bold]Zenoh Config Manager[/]"),
            Label(
                f"[dim]Network: {self.ctx.active_network} — Base stations connect to robots (star topology)[/]"
            ),
            Rule(),
            DataTable(id="zenoh_table"),
            Rule(),
            RichLog(id="zenoh_log", highlight=True, markup=True, max_lines=30),
        )
        yield Footer()

    def on_mount(self):
        self._endpoints = {}  # machine_name -> list of endpoint strings
        table = self.query_one("#zenoh_table", DataTable)
        table.add_column("Machine", key="machine")
        table.add_column("Role", key="role")
        table.add_column("Connect Endpoints", key="connect")
        table.add_column("Status", key="status")
        table.cursor_type = "row"
        self._build_table()

    def _build_table(self):
        table = self.query_one("#zenoh_table", DataTable)
        table.clear()
        log = self.query_one("#zenoh_log", RichLog)
        self._endpoints = {}

        for mname in sorted(self.ctx.runtime_config):
            mconf = self.ctx.runtime_config[mname]
            try:
                endpoints = generate_zenoh_endpoints(
                    self.ctx.topo, self.ctx.active_network, mname
                )
                self._endpoints[mname] = endpoints
            except KeyError:
                table.add_row(mname, mconf["role"], "?", "No address", key=mname)
                continue

            n = len(endpoints)
            connect_str = f"{n} robot(s)" if n else "none (listen only)"
            table.add_row(mname, mconf["role"], connect_str, "Ready", key=mname)

        log.write(
            f"[dim]Computed endpoints for {len(self._endpoints)} machine(s). Press [bold]d[/] to deploy, [bold]p[/] to preview.[/]"
        )

    def action_preview(self):
        table = self.query_one("#zenoh_table", DataTable)
        log = self.query_one("#zenoh_log", RichLog)
        try:
            from textual.coordinate import Coordinate

            row_idx = table.cursor_coordinate.row
            mname = str(table.get_cell_at(Coordinate(row_idx, 0)))
        except Exception:
            log.write("[red]No machine selected.[/]")
            return

        endpoints = self._endpoints.get(mname)
        if endpoints is None:
            log.write(f"[red]No config for {mname}.[/]")
            return

        log.write(f"\n[bold]Connect endpoints for {mname}:[/]")
        if endpoints:
            for ep in endpoints:
                log.write(f"  {ep}")
        else:
            log.write("  (none — listen only)")

    def action_deploy_all(self):
        log = self.query_one("#zenoh_log", RichLog)
        if not self._endpoints:
            log.write("[red]No machines to deploy to.[/]")
            return

        log.write(
            "[dim]Deploying configs (updating connect.endpoints in existing JSON5)...[/]"
        )

        def do_deploy():
            results = []
            machines_to_deploy = []
            for mname, endpoints in self._endpoints.items():
                mconf = self.ctx.runtime_config.get(mname)
                if not mconf or not mconf.get("online"):
                    results.append((mname, False, "Offline"))
                    continue
                machines_to_deploy.append((mname, mconf, endpoints))

            def deploy_one(item):
                mname, mconf, endpoints = item
                ok, msg = deploy_zenoh_config(mconf["user"], mconf["ip"], endpoints)
                return mname, ok, msg

            parallel_results = run_parallel(deploy_one, machines_to_deploy)
            for item, result in parallel_results:
                if isinstance(result, Exception):
                    results.append((item[0], False, str(result)))
                else:
                    results.append(result)

            return results

        def on_done(results):
            table = self.query_one("#zenoh_table", DataTable)
            for mname, ok, msg in sorted(results):
                status = f"[green]{msg}[/]" if ok else f"[red]{msg}[/]"
                log.write(f"  {mname}: {status}")
                # Update table status column
                try:
                    table.update_cell(mname, "status", msg)
                except Exception:
                    pass
            deployed = sum(1 for _, ok, _ in results if ok)
            log.write(f"[bold]Deployed {deployed}/{len(results)} configs.[/]")

        def bg():
            results = do_deploy()
            self.app.call_from_thread(on_done, results)

        threading.Thread(target=bg, daemon=True).start()


# ---- Main App ----
