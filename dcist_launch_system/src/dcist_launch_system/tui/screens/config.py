"""Config screen — topology editor."""

from __future__ import annotations

import pathlib
import threading

import yaml
from textual.app import ComposeResult
from textual.binding import Binding
from textual.containers import Vertical, VerticalScroll
from textual.screen import ModalScreen
from textual.widgets import (
    DataTable,
    Footer,
    Input,
    Label,
    RichLog,
    Rule,
)

from dcist_launch_system.fleet_helpers import (
    _DEFAULT_TOPOLOGY,
    filter_reachable,
)
from dcist_launch_system.tui.context import TuiContext


class EditMachineScreen(ModalScreen):
    """Modal dialog for editing or adding a single machine entry."""

    AUTO_FOCUS = "Input"
    BINDINGS = [
        Binding("escape", "cancel", "Cancel", priority=True),
        Binding("f5", "save", "Save [F5]", priority=True),
    ]
    DEFAULT_CSS = """
    EditMachineScreen { align: center middle; }
    EditMachineScreen > Vertical {
        width: 60;
        height: auto;
        border: thick $primary;
        background: $surface;
        padding: 1 2;
    }
    EditMachineScreen Input { width: 1fr; }
    EditMachineScreen #err_label { color: $error; }
    """

    def __init__(self, name=None, minfo=None, existing_names=None, **kwargs):
        super().__init__(**kwargs)
        self._name = name
        self._minfo = minfo or {}
        self._existing_names = existing_names or set()

    def compose(self) -> ComposeResult:
        addrs = self._minfo.get("addresses", {})
        title = f"Editing: {self._name}" if self._name else "Add New Machine"
        rows = [Label(f"[bold]{title}[/]")]
        sof = {"select_on_focus": False}
        if not self._name:
            rows.append(Input(placeholder="Name (e.g. newton)", id="ed_name", **sof))
        rows += [
            Input(
                value=self._minfo.get("role", "robot"),
                placeholder="Role (robot / base_station)",
                id="ed_role",
                **sof,
            ),
            Input(
                value=self._minfo.get("platform_id", ""),
                placeholder="Platform (e.g. smaug)",
                id="ed_platform",
                **sof,
            ),
            Input(
                value=addrs.get("mit_wifi", ""),
                placeholder="MIT WiFi IP (e.g. 10.29.x.x)",
                id="ed_mit_wifi",
                **sof,
            ),
            Input(
                value=addrs.get("silvus", ""),
                placeholder="Silvus IP (e.g. 192.168.100.x)",
                id="ed_silvus",
                **sof,
            ),
            Input(
                value=self._minfo.get("desc", ""),
                placeholder="Description",
                id="ed_desc",
                **sof,
            ),
            Label("[dim]Tab=next field  F5=save  Escape=cancel[/]"),
            Label("", id="err_label"),
        ]
        yield Vertical(*rows)

    def action_cancel(self):
        self.dismiss(None)

    def action_save(self):
        err = self.query_one("#err_label", Label)
        if self._name:
            name = self._name
        else:
            name = self.query_one("#ed_name", Input).value.strip()
            if not name:
                err.update("[red]Name is required.[/]")
                return
            if name in self._existing_names:
                err.update(f"[red]'{name}' already exists.[/]")
                return
        role = self.query_one("#ed_role", Input).value.strip() or "robot"
        if role not in ("robot", "base_station"):
            err.update("[red]Role must be 'robot' or 'base_station'.[/]")
            return
        self.dismiss(
            {
                "name": name,
                "role": role,
                "platform": self.query_one("#ed_platform", Input).value.strip(),
                "desc": self.query_one("#ed_desc", Input).value.strip(),
                "mit_wifi": self.query_one("#ed_mit_wifi", Input).value.strip(),
                "silvus": self.query_one("#ed_silvus", Input).value.strip(),
            }
        )


class ConfigScreen(ModalScreen):
    """Topology editor — manages machines in topology.yaml."""

    BINDINGS = [
        Binding("escape", "dismiss", "Back", priority=True),
        Binding("e", "edit_selected", "Edit [e]"),
        Binding("enter", "edit_selected", "Edit", show=False),
        Binding("f2", "add_machine_action", "Add [F2]", priority=True),
        Binding("f3", "recheck_action", "Recheck [F3]", priority=True),
        Binding("f4", "remove_selected", "Remove [F4]", priority=True),
        Binding("f5", "save_action", "Save [F5]", priority=True),
    ]

    DEFAULT_CSS = """
    ConfigScreen #config_table { height: auto; max-height: 16; }
    """

    def __init__(self, ctx: TuiContext):
        super().__init__()
        self.ctx = ctx

    def compose(self) -> ComposeResult:
        yield VerticalScroll(
            Label("[bold]Topology Editor[/]"),
            Label("[dim]e=edit, F2=add, F4=remove, F3=recheck, F5=save to YAML[/]"),
            Rule(),
            DataTable(id="config_table"),
            Rule(),
            RichLog(id="config_log", highlight=True, markup=True, max_lines=20),
        )
        yield Footer()

    def on_mount(self):
        table = self.query_one("#config_table", DataTable)
        table.add_column("", key="status")
        table.add_column("Name", key="name")
        table.add_column("Role", key="role")
        table.add_column("Platform", key="platform")
        table.add_column("MIT WiFi", key="mit_wifi")
        table.add_column("Silvus", key="silvus")
        table.add_column("User", key="user")
        table.add_column("Desc", key="desc")
        table.cursor_type = "row"
        self._refresh_config_table()

    def _refresh_config_table(self):
        table = self.query_one("#config_table", DataTable)
        table.clear()
        topo_machines = self.ctx.topo.get("machines", {})
        for mname in sorted(topo_machines):
            minfo = topo_machines[mname]
            addrs = minfo.get("addresses", {})
            rconf = self.ctx.runtime_config.get(mname, {})
            online = rconf.get("online")
            if online:
                dot = "[green]\u25cf[/]"
            elif rconf.get("ping"):
                dot = "[yellow]\u25cf[/]"
            elif online is False:
                dot = "[red]\u25cf[/]"
            else:
                dot = "[dim]\u25cb[/]"
            ssh_conf = self.ctx.topo.get("ssh", {})
            user = (
                ssh_conf.get("base_station_user", "rrg")
                if minfo.get("role") == "base_station"
                else ssh_conf.get("robot_user", "swarm")
            )
            table.add_row(
                dot,
                mname,
                minfo.get("role", "?"),
                minfo.get("platform_id", "") or "—",
                addrs.get("mit_wifi", "—"),
                addrs.get("silvus", "—"),
                user,
                minfo.get("desc", ""),
                key=mname,
            )

    def _get_selected_name(self):
        table = self.query_one("#config_table", DataTable)
        try:
            from textual.coordinate import Coordinate

            row_idx = table.cursor_coordinate.row
            return str(table.get_cell_at(Coordinate(row_idx, 1)))
        except Exception:
            return None

    def _apply_result(self, result, original_name=None):
        """Apply an EditMachineScreen result dict to self.ctx.topo and self.ctx.runtime_config."""
        if not result:
            return
        log = self.query_one("#config_log", RichLog)
        name = result["name"]
        role = result["role"]
        platform = result["platform"]
        desc = result["desc"]
        mit_wifi = result["mit_wifi"]
        silvus = result["silvus"]
        topo_machines = self.ctx.topo.setdefault("machines", {})
        addrs = {}
        if mit_wifi:
            addrs["mit_wifi"] = mit_wifi
        if silvus:
            addrs["silvus"] = silvus
        # Preserve other networks (e.g. penn_wifi) from the original entry
        ref_name = original_name or name
        if ref_name in topo_machines:
            for net, ip in topo_machines[ref_name].get("addresses", {}).items():
                if net not in ("mit_wifi", "silvus"):
                    addrs[net] = ip
        entry = {"role": role, "addresses": addrs}
        if platform:
            entry["platform_id"] = platform
        if desc:
            entry["desc"] = desc
        topo_machines[name] = entry
        if self.ctx.active_network in addrs:
            ssh_conf = self.ctx.topo.get("ssh", {})
            user = (
                ssh_conf.get("base_station_user", "rrg")
                if role == "base_station"
                else ssh_conf.get("robot_user", "swarm")
            )
            self.ctx.runtime_config[name] = {
                "name": name,
                "role": role,
                "ip": addrs[self.ctx.active_network],
                "user": user,
                "desc": desc,
                "platform_id": platform or "",
            }
        log.write(f"[green]Updated {name}[/]")
        self._refresh_config_table()

    def action_edit_selected(self):
        name = self._get_selected_name()
        if not name:
            return
        minfo = self.ctx.topo.get("machines", {}).get(name, {})
        self.app.push_screen(
            EditMachineScreen(name, minfo),
            lambda r: self._apply_result(r, original_name=name),
        )

    def action_add_machine_action(self):
        existing = set(self.ctx.topo.get("machines", {}).keys())
        self.app.push_screen(
            EditMachineScreen(None, {}, existing_names=existing), self._apply_result
        )

    def action_remove_selected(self):
        log = self.query_one("#config_log", RichLog)
        name = self._get_selected_name()
        if not name:
            log.write("[yellow]No machine selected.[/]")
            return
        topo_machines = self.ctx.topo.get("machines", {})
        if name in topo_machines:
            del topo_machines[name]
        if name in self.ctx.runtime_config:
            del self.ctx.runtime_config[name]
        self.ctx._deleted_machines.add(name)
        log.write(f"Removed [bold]{name}[/]. Press F5 to save permanently.")
        self._refresh_config_table()

    def action_recheck_action(self):
        log = self.query_one("#config_log", RichLog)
        log.write("[dim]Rechecking connectivity...[/]")

        def do_check():
            machines = list(self.ctx.runtime_config.values())
            reachable, unreachable = filter_reachable(machines)
            return reachable + unreachable

        def on_done(all_machines):
            n_online = sum(1 for m in all_machines if m.get("online"))
            log.write(f"[green]{n_online}/{len(all_machines)} machines online.[/]")
            self._refresh_config_table()

        def bg():
            results = do_check()
            self.app.call_from_thread(on_done, results)

        threading.Thread(target=bg, daemon=True).start()

    def action_save_action(self):
        log = self.query_one("#config_log", RichLog)
        topo_machines = self.ctx.topo.get("machines", {})
        for mname in list(self.ctx._deleted_machines):
            if mname in topo_machines:
                del topo_machines[mname]
        self.ctx._deleted_machines.clear()
        self.ctx.topo["machines"] = topo_machines
        topo_path = (
            pathlib.Path(self.ctx.topology_path)
            if self.ctx.topology_path
            else _DEFAULT_TOPOLOGY
        )
        try:
            with open(topo_path, "w") as f:
                yaml.dump(self.ctx.topo, f, default_flow_style=False, sort_keys=False)
            log.write(f"[green]Saved to {topo_path}[/]")
        except OSError as e:
            log.write(f"[red]Failed to save: {e}[/]")


# ---- Launch Screen ----
