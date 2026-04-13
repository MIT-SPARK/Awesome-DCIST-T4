"""Transfer screen — copy experiments between machines."""

from __future__ import annotations

import threading

from textual.app import ComposeResult
from textual.binding import Binding
from textual.containers import Vertical
from textual.screen import ModalScreen
from textual.widgets import (
    Footer,
    Label,
    ProgressBar,
    RichLog,
    Rule,
    SelectionList,
)

from dcist_launch_system.fleet_helpers import (
    rsync_transfer,
)
from dcist_launch_system.tui.context import TuiContext


class TransferScreen(ModalScreen):
    BINDINGS = [
        Binding("escape", "dismiss", "Back", priority=True),
        Binding("t", "start_transfer", "Start Transfer", priority=True),
    ]

    def __init__(self, ctx: TuiContext, src_machine, experiment):
        super().__init__()
        self.ctx = ctx
        self.src_machine = src_machine
        self.experiment = experiment
        self._transferring = False

    def compose(self) -> ComposeResult:
        others = [
            (m["name"], m["name"])
            for m in self.ctx.runtime_config.values()
            if m["name"] != self.src_machine and m.get("online", False)
        ]
        yield Vertical(
            Label(
                f"[bold]Transfer[/] {self.src_machine}:{self.experiment} -> select destinations"
            ),
            Rule(),
            SelectionList(*others, id="dest_select"),
            Rule(),
            Vertical(id="progress_area"),
            RichLog(id="transfer_log", highlight=True, markup=True, max_lines=100),
        )
        yield Footer()

    def action_start_transfer(self):
        self._do_transfer()

    def _do_transfer(self):
        if self._transferring:
            return
        sel = self.query_one("#dest_select", SelectionList)
        selected_names = list(sel.selected)
        if not selected_names:
            self.query_one("#transfer_log", RichLog).write(
                "[yellow]Select at least one destination.[/]"
            )
            return

        self._transferring = True
        log = self.query_one("#transfer_log", RichLog)
        src = self.ctx.runtime_config[self.src_machine]
        src_path = f"{self.ctx.output_root}/{self.experiment}"
        dest_name = f"prior_{self.experiment}"

        log.write(f"[yellow]Excluding rosbags. Saving as {dest_name}[/]")

        # Build progress bars in the UI
        area = self.query_one("#progress_area", Vertical)
        area.remove_children()
        for dname in selected_names:
            area.mount(
                Vertical(
                    Label(f"  {dname}:"),
                    ProgressBar(total=100, show_eta=False, id=f"pb_{dname}"),
                    id=f"pbrow_{dname}",
                )
            )

        def do_transfers():
            try:
                for i, dname in enumerate(selected_names):
                    dst = self.ctx.runtime_config[dname]
                    dst_path = f"{self.ctx.output_root}/{dest_name}"
                    self.app.call_from_thread(
                        log.write, f"Transferring to [bold]{dname}[/]..."
                    )

                    def make_cb(name):
                        def cb(pct):
                            self.app.call_from_thread(self._update_progress, name, pct)

                        return cb

                    ok, method, msg = rsync_transfer(
                        src["user"],
                        src["ip"],
                        src_path,
                        dst["user"],
                        dst["ip"],
                        dst_path,
                        exclude=["recorded_data/"],
                        progress_callback=make_cb(dname),
                    )
                    if ok:
                        self.app.call_from_thread(self._update_progress, dname, 100)
                        self.app.call_from_thread(
                            log.write, f"  [green]{dname}: OK ({method})[/]"
                        )
                    else:
                        self.app.call_from_thread(
                            log.write, f"  [red]{dname}: FAIL ({method}) {msg}[/]"
                        )
                self.app.call_from_thread(log.write, "[bold]Transfer complete.[/]")
            finally:
                self._transferring = False

        threading.Thread(target=do_transfers, daemon=True).start()

    def _update_progress(self, dname, pct):
        try:
            bar = self.query_one(f"#pb_{dname}", ProgressBar)
            bar.update(progress=pct)
        except Exception:
            pass


# ---- Verify Screen ----
