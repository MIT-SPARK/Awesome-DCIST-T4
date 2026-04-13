"""Bandwidth test selection screen."""

from __future__ import annotations

import threading

from textual.app import ComposeResult
from textual.binding import Binding
from textual.containers import Vertical
from textual.screen import ModalScreen
from textual.widgets import (
    Label,
    RichLog,
    Rule,
    SelectionList,
)

from dcist_launch_system.fleet_helpers import (
    check_iperf3,
    run_iperf3_test,
)


class BandwidthSelectScreen(ModalScreen):
    """Select robots, run iperf3 tests with live progress, dismiss with results."""

    AUTO_FOCUS = "SelectionList"
    BINDINGS = [
        Binding("escape", "close", "Cancel/Close", priority=True),
        Binding("f5", "run_test", "Run [F5]", priority=True),
    ]
    DEFAULT_CSS = """
    BandwidthSelectScreen { align: center middle; }
    BandwidthSelectScreen > Vertical {
        width: 60;
        height: auto;
        max-height: 35;
        border: thick $primary;
        background: $surface;
        padding: 1 2;
    }
    BandwidthSelectScreen #bw_progress { height: 15; }
    """

    def __init__(self, machines, server_ip, **kwargs):
        super().__init__(**kwargs)
        self._machines = machines
        self._server_ip = server_ip
        self._results = []
        self._done = False

    def compose(self) -> ComposeResult:
        options = [(m["name"], m["name"], True) for m in self._machines]
        yield Vertical(
            Label("[bold]Bandwidth Test[/]"),
            Label("[dim]Space=toggle  F5=run  Escape=cancel[/]", id="bw_hint"),
            Rule(),
            SelectionList(*options, id="bw_select"),
            RichLog(id="bw_progress", highlight=True, markup=True, max_lines=100),
        )

    def on_mount(self):
        self.query_one("#bw_progress", RichLog).display = False

    def action_close(self):
        self.dismiss(self._results or None)

    def action_run_test(self):
        if self._done:
            return
        sel = self.query_one("#bw_select", SelectionList)
        chosen = [m for m in self._machines if m["name"] in set(sel.selected)]
        if not chosen:
            return

        # Switch to progress view
        sel.display = False
        hint = self.query_one("#bw_hint", Label)
        hint.update("[dim]Escape to close when finished[/]")
        plog = self.query_one("#bw_progress", RichLog)
        plog.display = True
        plog.write(
            f"[dim]Server: {self._server_ip}  →  {', '.join(m['name'] for m in chosen)}[/]"
        )

        def emit(r):
            name = r.get("name", "?")
            if "error" in r:
                plog.write(f"[red]{name}: {r['error']}[/]")
            else:
                mbps = r.get("mbps", 0)
                color = "green" if mbps > 10 else "yellow" if mbps > 1 else "red"
                tunnel_warn = (
                    "  [yellow](via SSH tunnel — direct port blocked)[/]"
                    if r.get("tunneled")
                    else ""
                )
                plog.write(
                    f"[{color}]{name}: {mbps:.1f} Mbps[/]"
                    f"  retransmits={r.get('retransmits', 0)}"
                    f"  ({r.get('duration', 0):.1f}s)"
                    f"{tunnel_warn}"
                )
            self._results.append(r)

        def all_done():
            plog.write("[dim]Done — press Escape to close[/]")
            self._done = True

        def bg():
            for m in chosen:
                name = m["name"]
                self.app.call_from_thread(plog.write, f"[bold]{name}[/]")
                if not check_iperf3(m["user"], m["ip"]):
                    r = {"name": name, "error": "iperf3 not installed"}
                else:

                    def on_interval(t_end, mbps, retransmits, _name=name):
                        color = (
                            "green" if mbps > 10 else "yellow" if mbps > 1 else "red"
                        )
                        retr = f"  retr={retransmits}" if retransmits else ""
                        self.app.call_from_thread(
                            plog.write,
                            f"  [{color}]{t_end:.0f}s  {mbps:.0f} Mbits/sec{retr}[/]",
                        )

                    r = run_iperf3_test(
                        self._server_ip,
                        m["user"],
                        m["ip"],
                        interval_cb=on_interval,
                    )
                    r["name"] = name
                self.app.call_from_thread(emit, r)
            self.app.call_from_thread(all_done)

        threading.Thread(target=bg, daemon=True).start()


# ---- Monitor Screen ----
