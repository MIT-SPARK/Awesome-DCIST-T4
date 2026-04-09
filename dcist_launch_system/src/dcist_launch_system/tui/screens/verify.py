"""Verify screen — checksum comparison across fleet."""

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
    hash_remote_experiment,
    run_parallel,
)
from dcist_launch_system.tui.context import TuiContext


class VerifyScreen(ModalScreen):
    BINDINGS = [Binding("escape", "dismiss", "Back", priority=True)]

    def __init__(self, ctx: TuiContext, experiment):
        super().__init__()
        self.ctx = ctx
        self.experiment = experiment

    def compose(self) -> ComposeResult:
        yield Vertical(
            Label(f"[bold]Verify:[/] {self.experiment}"),
            Rule(),
            DataTable(id="verify_table"),
            RichLog(id="verify_log", highlight=True, markup=True, max_lines=50),
            Footer(),
        )

    def on_mount(self):
        log = self.query_one("#verify_log", RichLog)
        log.write("[dim]Computing checksums across fleet...[/]")

        def do_verify():
            machines = [
                m for m in self.ctx.runtime_config.values() if m.get("online", False)
            ]

            def fetch(m):
                return hash_remote_experiment(
                    m["user"], m["ip"], self.ctx.output_root, self.experiment
                )

            return run_parallel(fetch, machines)

        def on_done(results):
            table = self.query_one("#verify_table", DataTable)
            machine_hashes = {
                m["name"]: h for m, h in results if isinstance(h, dict) and h
            }
            if not machine_hashes:
                log.write("[yellow]No hash data returned.[/]")
                return

            all_files = sorted(set(f for h in machine_hashes.values() for f in h))
            mnames = sorted(machine_hashes.keys())

            table.add_column("File", key="file")
            for mn in mnames:
                table.add_column(mn, key=mn)

            diffs = 0
            for fpath in all_files:
                hashes = {mn: machine_hashes[mn].get(fpath) for mn in mnames}
                unique = set(v for v in hashes.values() if v)
                consistent = len(unique) <= 1 and all(v for v in hashes.values())
                row = [fpath]
                for mn in mnames:
                    h = hashes[mn]
                    if h is None:
                        row.append("MISSING")
                        consistent = False
                    elif consistent:
                        row.append("OK")
                    else:
                        row.append(h[:8])
                if not consistent:
                    diffs += 1
                table.add_row(*row)

            if diffs == 0:
                log.write("[bold green]All files consistent.[/]")
            else:
                log.write(f"[bold red]{diffs} file(s) differ.[/]")

        def bg():
            results = do_verify()
            self.app.call_from_thread(on_done, results)

        threading.Thread(target=bg, daemon=True).start()


# ---- Config Screen ----
