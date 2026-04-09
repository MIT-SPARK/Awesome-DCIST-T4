# fleet-adt4 Modularize Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Refactor the 3119-line `bin/fleet-adt4` monolith into an importable Python package `dcist_launch_system.tui`, split across focused modules (one per screen), with a thin entry-point shim and automated tests.

**Architecture:** Option 1 (thin entry point) is done first — move all code into `tui/__init__.py`, make `bin/fleet-adt4` a 10-line shim. Then Option 2 breaks the nested-closure architecture: a `TuiContext` dataclass carries all shared mutable state and each screen moves to its own file. CLI commands stay in `tui/_cli.py` (or keep in `tui/__init__.py`).

**Tech Stack:** Python 3.11+, Textual (TUI framework), Click (CLI), pytest + pytest-asyncio, textual.testing.Pilot

---

## File Map

### New files
| File | Responsibility |
|---|---|
| `src/dcist_launch_system/tui/__init__.py` | Package init — exports `cli` entry point |
| `src/dcist_launch_system/tui/_cli.py` | Click CLI group + all subcommands (status, maps, transfer, verify, launch, monitor, zenoh) |
| `src/dcist_launch_system/tui/context.py` | `TuiContext` dataclass + `_fuzzy_match()` + `_check_sessions_and_prompt()` |
| `src/dcist_launch_system/tui/app.py` | `FleetApp`, `_launch_tui()`, `_start_fleet_zenohd()`, `_stop_fleet_zenohd()` |
| `src/dcist_launch_system/tui/screens/__init__.py` | Re-exports all screen classes |
| `src/dcist_launch_system/tui/screens/confirm.py` | `ConfirmScreen` |
| `src/dcist_launch_system/tui/screens/maps.py` | `PriorMapSelector`, `MapsScreen` |
| `src/dcist_launch_system/tui/screens/transfer.py` | `TransferScreen` |
| `src/dcist_launch_system/tui/screens/verify.py` | `VerifyScreen` |
| `src/dcist_launch_system/tui/screens/config.py` | `EditMachineScreen`, `ConfigScreen` |
| `src/dcist_launch_system/tui/screens/launch.py` | `LaunchScreen` |
| `src/dcist_launch_system/tui/screens/bandwidth.py` | `BandwidthSelectScreen` |
| `src/dcist_launch_system/tui/screens/monitor.py` | `MonitorScreen` |
| `src/dcist_launch_system/tui/screens/zenoh_screen.py` | `ZenohScreen` |
| `tests/tui/conftest.py` | Shared fixtures (mock `TuiContext`) |
| `tests/tui/test_import.py` | Import/module structure smoke tests |
| `tests/tui/test_context.py` | `TuiContext` unit tests |
| `tests/tui/test_screens.py` | Textual `run_test()` compose/mount tests |

### Modified files
| File | Change |
|---|---|
| `bin/fleet-adt4` | Replace with ~10-line shim calling `from dcist_launch_system.tui import cli; cli()` |

---

## Key Architecture: TuiContext

All screens currently capture these variables from the enclosing `_launch_tui()` closure:

```python
@dataclass
class TuiContext:
    topo: dict                          # loaded topology
    active_network: str                 # e.g. "mit_wifi"
    topology_path: str | None           # path to topology.yaml
    output_root: str                    # e.g. "~/adt4_output"
    runtime_config: dict[str, dict]     # mutable: machine state + connectivity
    _deleted_machines: set[str]         # track removals for save
    _local_ips: set[str]                # local machine IPs
    _fleet_zenoh: dict                  # {"proc": ..., "config": ...}
```

All screens receive `ctx: TuiContext` in `__init__` and replace closure-variable references:
- `topo` → `self.ctx.topo`
- `active_network` → `self.ctx.active_network`
- `runtime_config` → `self.ctx.runtime_config`
- `_deleted_machines` → `self.ctx._deleted_machines`
- `_local_ips` → `self.ctx._local_ips`
- `_fleet_zenoh` → `self.ctx._fleet_zenoh`
- `_fuzzy_match(...)` → `self.ctx.fuzzy_match(...)`

---

## Task 1: Option 1 — Move all code to `tui/__init__.py`

**Files:**
- Create: `src/dcist_launch_system/tui/__init__.py`
- Modify: `bin/fleet-adt4`

This task does NOT restructure code — it's a pure move with two small adaptations:
1. Wrap re-exec logic in a `_maybe_reexec()` function (avoids running on import)
2. Remove the `sys.path.insert` hack (package is properly installed, and tests import from source)

- [ ] **Step 1: Create `tui/__init__.py` with the full content**

  Copy all of `bin/fleet-adt4` (lines 1–3119) into `src/dcist_launch_system/tui/__init__.py` with these changes:

  a) **Remove shebang** (line 1: `#!/usr/bin/env python3`) — not valid in a module file.

  b) **Wrap re-exec block in a function** (currently lines 12–19 at module level):

  ```python
  # Before (module-level — runs on import):
  if not os.environ.get("_FLEET_ADT4_REEXEC"):
      _adt4_env = os.environ.get("ADT4_ENV", "")
      if _adt4_env:
          _spark_env = pathlib.Path(_adt4_env) / "spark_env"
          _spark_python = _spark_env / "bin" / "python3"
          if _spark_python.exists() and sys.prefix != str(_spark_env):
              os.environ["_FLEET_ADT4_REEXEC"] = "1"
              os.execv(str(_spark_python), [str(_spark_python)] + sys.argv)
  ```

  ```python
  # After (called only from the entry shim):
  def _maybe_reexec():
      """Re-exec with spark_env Python if ADT4_ENV points to one."""
      if not os.environ.get("_FLEET_ADT4_REEXEC"):
          _adt4_env = os.environ.get("ADT4_ENV", "")
          if _adt4_env:
              _spark_env = pathlib.Path(_adt4_env) / "spark_env"
              _spark_python = _spark_env / "bin" / "python3"
              if _spark_python.exists() and sys.prefix != str(_spark_env):
                  os.environ["_FLEET_ADT4_REEXEC"] = "1"
                  os.execv(str(_spark_python), [str(_spark_python)] + sys.argv)
  ```

  c) **Remove `sys.path.insert` block** (currently lines 31–33). The package is installed; `import dcist_launch_system.fleet_helpers` works without it. The shim will handle this if needed for a dev install.

  d) The `if __name__ == "__main__": cli()` at the bottom stays as-is.

- [ ] **Step 2: Replace `bin/fleet-adt4` with a thin shim**

  ```python
  #!/usr/bin/env python3
  """fleet-adt4 entry point — delegates to dcist_launch_system.tui package."""
  import pathlib
  import sys

  # Dev install: add src/ to path so the package is importable
  _src = pathlib.Path(__file__).resolve().parent.parent / "src"
  if _src.is_dir() and str(_src) not in sys.path:
      sys.path.insert(0, str(_src))

  from dcist_launch_system.tui import _maybe_reexec, cli

  _maybe_reexec()
  cli()
  ```

  Then make it executable:
  ```bash
  chmod +x bin/fleet-adt4
  ```

- [ ] **Step 3: Verify the shim works**

  ```bash
  cd /home/rrg/dcist_ws/src/awesome_dcist_t4/dcist_launch_system
  python bin/fleet-adt4 --help
  ```

  Expected: prints the `fleet-adt4` click help with subcommands (status, maps, transfer, verify, launch, monitor, zenoh).

- [ ] **Step 4: Verify import works**

  ```bash
  cd /home/rrg/dcist_ws/src/awesome_dcist_t4/dcist_launch_system/src
  python -c "from dcist_launch_system.tui import cli; print('OK')"
  ```

  Expected: prints `OK` without executing any re-exec code or TUI startup.

- [ ] **Step 5: Run existing tests**

  ```bash
  cd /home/rrg/dcist_ws/src/awesome_dcist_t4/dcist_launch_system
  python -m pytest tests/ -x -q
  ```

  Expected: all existing tests pass (they test `fleet_helpers.py`, not the TUI).

- [ ] **Step 6: Commit**

  ```bash
  git add src/dcist_launch_system/tui/__init__.py bin/fleet-adt4
  git commit -m "fleet-adt4: move TUI to dcist_launch_system.tui package (option 1)"
  ```

---

## Task 2: Create `TuiContext` dataclass

**Files:**
- Create: `src/dcist_launch_system/tui/context.py`
- Create: `tests/tui/conftest.py`
- Create: `tests/tui/test_context.py`

- [ ] **Step 1: Create `context.py`**

  ```python
  """Shared mutable state passed to all TUI screens."""
  from __future__ import annotations

  from dataclasses import dataclass, field
  from typing import Any


  @dataclass
  class TuiContext:
      """Carries all mutable state shared between TUI screens.

      Replaces closure captures from the original monolithic _launch_tui().
      """
      topo: dict
      active_network: str
      topology_path: str | None
      output_root: str
      runtime_config: dict[str, dict] = field(default_factory=dict)
      _deleted_machines: set[str] = field(default_factory=set)
      _local_ips: set[str] = field(default_factory=set)
      _fleet_zenoh: dict[str, Any] = field(default_factory=lambda: {"proc": None, "config": None})

      @staticmethod
      def fuzzy_match(query: str, text: str) -> bool:
          """Subsequence fuzzy match — all chars of query appear in order in text."""
          if not query:
              return True
          query = query.lower()
          text = text.lower()
          qi = 0
          for ch in text:
              if ch == query[qi]:
                  qi += 1
                  if qi == len(query):
                      return True
          return False
  ```

- [ ] **Step 2: Create `tests/tui/__init__.py` and `tests/tui/conftest.py`**

  ```bash
  mkdir -p tests/tui && touch tests/tui/__init__.py
  ```

  ```python
  # tests/tui/conftest.py
  import pytest
  from dcist_launch_system.tui.context import TuiContext


  @pytest.fixture
  def sample_topo():
      return {
          "networks": {
              "mit_wifi": {"subnet": "10.29.0.0/16"},
              "silvus": {"subnet": "192.168.100.0/24"},
          },
          "machines": {
              "alpha": {
                  "role": "robot",
                  "platform_id": "smaug",
                  "desc": "Test Robot A",
                  "addresses": {"mit_wifi": "10.29.1.1", "silvus": "192.168.100.10"},
              },
              "beta": {
                  "role": "robot",
                  "platform_id": "topaz",
                  "desc": "Test Robot B",
                  "addresses": {"mit_wifi": "10.29.1.2"},
              },
              "gamma": {
                  "role": "base_station",
                  "desc": "Test Base Station",
                  "addresses": {"mit_wifi": "10.29.1.3", "silvus": "192.168.100.100"},
              },
          },
          "ssh": {"robot_user": "swarm", "base_station_user": "rrg"},
          "zenoh": {"port": 7447, "connect_network": "silvus"},
          "fleet_defaults": {
              "spot_platforms": ["smaug", "topaz"],
              "sessions": {
                  "mapping": {"spot": "spot-default", "phoenix": "phoenix-default"},
                  "relocalize": {"spot": "spot_relocalize-relocalize", "phoenix": "phoenix_relocalize-relocalize"},
              },
              "base_station_session": "base_station_heracles-default",
              "output_root": "~/adt4_output",
          },
      }


  @pytest.fixture
  def ctx(sample_topo):
      """Minimal TuiContext for screen tests."""
      machines = {
          "alpha": {"name": "alpha", "role": "robot", "platform_id": "smaug", "ip": "10.29.1.1", "online": False},
          "beta": {"name": "beta", "role": "robot", "platform_id": "topaz", "ip": "10.29.1.2", "online": False},
          "gamma": {"name": "gamma", "role": "base_station", "platform_id": None, "ip": "10.29.1.3", "online": False},
      }
      return TuiContext(
          topo=sample_topo,
          active_network="mit_wifi",
          topology_path=None,
          output_root="~/adt4_output",
          runtime_config=machines,
      )
  ```

- [ ] **Step 3: Write tests for TuiContext**

  ```python
  # tests/tui/test_context.py
  from dcist_launch_system.tui.context import TuiContext


  def test_construction(sample_topo):
      ctx = TuiContext(
          topo=sample_topo,
          active_network="mit_wifi",
          topology_path=None,
          output_root="~/adt4_output",
      )
      assert ctx.active_network == "mit_wifi"
      assert ctx.runtime_config == {}
      assert ctx._deleted_machines == set()
      assert ctx._fleet_zenoh == {"proc": None, "config": None}


  def test_fuzzy_match_basic():
      assert TuiContext.fuzzy_match("abc", "aXbYcZ") is True
      assert TuiContext.fuzzy_match("abc", "aXbY") is False
      assert TuiContext.fuzzy_match("", "anything") is True
      assert TuiContext.fuzzy_match("ABC", "aXbYcZ") is True   # case-insensitive


  def test_fuzzy_match_exact():
      assert TuiContext.fuzzy_match("euclid", "euclid") is True
      assert TuiContext.fuzzy_match("euc", "euclid") is True
      assert TuiContext.fuzzy_match("xyz", "euclid") is False


  def test_runtime_config_mutation(ctx):
      ctx.runtime_config["alpha"]["online"] = True
      assert ctx.runtime_config["alpha"]["online"] is True


  def test_deleted_machines_mutation(ctx):
      ctx._deleted_machines.add("alpha")
      assert "alpha" in ctx._deleted_machines
  ```

- [ ] **Step 4: Run new tests**

  ```bash
  cd /home/rrg/dcist_ws/src/awesome_dcist_t4/dcist_launch_system
  python -m pytest tests/tui/test_context.py -v
  ```

  Expected: 5 tests pass.

- [ ] **Step 5: Commit**

  ```bash
  git add src/dcist_launch_system/tui/context.py tests/tui/
  git commit -m "fleet-adt4 tui: add TuiContext dataclass + tests"
  ```

---

## Task 3: Create `tui/screens/` package with `ConfirmScreen`

**Files:**
- Create: `src/dcist_launch_system/tui/screens/__init__.py`
- Create: `src/dcist_launch_system/tui/screens/confirm.py`

This task establishes the screens package and extracts the simplest screen first.

- [ ] **Step 1: Create `screens/__init__.py`**

  ```python
  # src/dcist_launch_system/tui/screens/__init__.py
  from .confirm import ConfirmScreen

  __all__ = ["ConfirmScreen"]
  ```

- [ ] **Step 2: Create `screens/confirm.py`**

  Extract `ConfirmScreen` from `tui/__init__.py` (currently inside `_launch_tui()`). This screen has NO shared state — it takes `message`, `yes_label`, `no_label` args and returns a `bool`. No `TuiContext` needed.

  ```python
  # src/dcist_launch_system/tui/screens/confirm.py
  from textual.app import ComposeResult
  from textual.binding import Binding
  from textual.containers import Vertical
  from textual.screen import ModalScreen
  from textual.widgets import Button, Label, Rule


  class ConfirmScreen(ModalScreen[bool]):
      BINDINGS = [
          Binding("y", "confirm_yes", "Yes", priority=True),
          Binding("n", "confirm_no", "No", priority=True),
          Binding("escape", "confirm_no", "Cancel", priority=True),
      ]

      def __init__(self, message: str, yes_label: str = "Yes, Proceed (y)", no_label: str = "No, Cancel (n)"):
          super().__init__()
          self.message = message
          self._yes_label = yes_label
          self._no_label = no_label

      def compose(self) -> ComposeResult:
          yield Vertical(
              Label(self.message),
              Rule(),
              Button(self._yes_label, variant="error", id="btn_yes"),
              Button(self._no_label, id="btn_no"),
          )

      def on_button_pressed(self, event: Button.Pressed) -> None:
          self.dismiss(event.button.id == "btn_yes")

      def action_confirm_yes(self) -> None:
          self.dismiss(True)

      def action_confirm_no(self) -> None:
          self.dismiss(False)
  ```

  > **Note:** Copy the exact implementation from `tui/__init__.py`. The snippet above is illustrative — use the actual code.

- [ ] **Step 3: Update `tui/__init__.py`** to import `ConfirmScreen` from the new module instead of defining it inline. Find the `class ConfirmScreen` definition inside `_launch_tui()` (around line 633) and replace with:

  ```python
  from dcist_launch_system.tui.screens.confirm import ConfirmScreen
  ```

  Place this import at the top of `_launch_tui()` alongside the other local imports.

- [ ] **Step 4: Verify still works**

  ```bash
  python bin/fleet-adt4 --help
  ```

  Expected: help text prints normally.

- [ ] **Step 5: Commit**

  ```bash
  git add src/dcist_launch_system/tui/screens/ src/dcist_launch_system/tui/__init__.py
  git commit -m "fleet-adt4 tui: extract ConfirmScreen to screens/confirm.py"
  ```

---

## Task 4: Extract `MapsScreen` and `PriorMapSelector`

**Files:**
- Create: `src/dcist_launch_system/tui/screens/maps.py`
- Modify: `src/dcist_launch_system/tui/screens/__init__.py`
- Modify: `src/dcist_launch_system/tui/__init__.py`

`MapsScreen` (lines 819–931) uses: `runtime_config`, `topo`, `active_network`, `output_root`.
`PriorMapSelector` (lines 710–817) uses: `runtime_config`, `active_network`.

- [ ] **Step 1: Create `screens/maps.py`**

  The file receives `ctx: TuiContext` in `__init__`. Replace all closure captures:

  ```python
  # src/dcist_launch_system/tui/screens/maps.py
  from __future__ import annotations

  import threading
  from textual.app import ComposeResult
  from textual.binding import Binding
  from textual.containers import Horizontal, Vertical, VerticalScroll
  from textual.screen import Screen
  from textual.widgets import DataTable, Footer, Header, Label, RichLog, Rule, Static

  from dcist_launch_system.fleet_helpers import (
      filter_reachable, list_remote_experiments, _quote_path,
  )
  from dcist_launch_system.tui.context import TuiContext


  class PriorMapSelector(Static):
      """Widget for selecting a prior map from the fleet."""

      def __init__(self, ctx: TuiContext):
          super().__init__()
          self.ctx = ctx
          # ... rest of __init__

      # ... rest of PriorMapSelector methods, using self.ctx.runtime_config etc.


  class MapsScreen(Screen):
      BINDINGS = [
          Binding("r", "refresh", "Refresh"),
          Binding("t", "transfer", "Transfer"),
          Binding("v", "verify", "Verify"),
          Binding("escape", "app.pop_screen", "Back"),
      ]

      def __init__(self, ctx: TuiContext):
          super().__init__()
          self.ctx = ctx

      # ... all methods, using self.ctx.* instead of closure vars
  ```

  > **Note:** Copy the exact implementations from `tui/__init__.py`. The above is the scaffold — fill in the bodies.

- [ ] **Step 2: Update `screens/__init__.py`**

  ```python
  from .confirm import ConfirmScreen
  from .maps import MapsScreen, PriorMapSelector

  __all__ = ["ConfirmScreen", "MapsScreen", "PriorMapSelector"]
  ```

- [ ] **Step 3: Update `_launch_tui()` in `tui/__init__.py`**

  Replace `class MapsScreen` and `class PriorMapSelector` definitions with:

  ```python
  from dcist_launch_system.tui.screens.maps import MapsScreen, PriorMapSelector
  ```

  Update the call site (where `MapsScreen()` is instantiated) to pass `ctx`:

  ```python
  def action_show_maps(self):
      self.push_screen(MapsScreen(ctx))
  ```

  (This is inside `FleetApp.action_show_maps` which is still in `__init__.py` at this stage.)

- [ ] **Step 4: Verify**

  ```bash
  python bin/fleet-adt4 --help
  python -c "from dcist_launch_system.tui.screens import MapsScreen; print('OK')"
  ```

- [ ] **Step 5: Commit**

  ```bash
  git add src/dcist_launch_system/tui/screens/maps.py src/dcist_launch_system/tui/screens/__init__.py src/dcist_launch_system/tui/__init__.py
  git commit -m "fleet-adt4 tui: extract MapsScreen + PriorMapSelector"
  ```

---

## Task 5: Extract `TransferScreen` and `VerifyScreen`

**Files:**
- Create: `src/dcist_launch_system/tui/screens/transfer.py`
- Create: `src/dcist_launch_system/tui/screens/verify.py`
- Modify: `src/dcist_launch_system/tui/screens/__init__.py`
- Modify: `src/dcist_launch_system/tui/__init__.py`

`TransferScreen` (lines 935–1043) uses: `runtime_config`, `active_network`.
`VerifyScreen` (lines 1047–1119) uses: `runtime_config`.
Both are pushed from `MapsScreen`, which already has `ctx`.

- [ ] **Step 1: Create `screens/transfer.py`** with `TransferScreen(ctx: TuiContext, source_machine: str, experiment: str)`. Replace all closure captures with `self.ctx.*`.

- [ ] **Step 2: Create `screens/verify.py`** with `VerifyScreen(ctx: TuiContext, source_machine: str, experiment: str)`. Replace all closure captures with `self.ctx.*`.

- [ ] **Step 3: Update `screens/__init__.py`**

  ```python
  from .confirm import ConfirmScreen
  from .maps import MapsScreen, PriorMapSelector
  from .transfer import TransferScreen
  from .verify import VerifyScreen

  __all__ = ["ConfirmScreen", "MapsScreen", "PriorMapSelector", "TransferScreen", "VerifyScreen"]
  ```

- [ ] **Step 4: Update `tui/__init__.py`** — replace `TransferScreen` and `VerifyScreen` class definitions with imports, update push_screen() call sites to pass `ctx`.

- [ ] **Step 5: Verify**

  ```bash
  python bin/fleet-adt4 --help
  python -c "from dcist_launch_system.tui.screens import TransferScreen, VerifyScreen; print('OK')"
  ```

- [ ] **Step 6: Commit**

  ```bash
  git add src/dcist_launch_system/tui/screens/transfer.py src/dcist_launch_system/tui/screens/verify.py src/dcist_launch_system/tui/screens/__init__.py src/dcist_launch_system/tui/__init__.py
  git commit -m "fleet-adt4 tui: extract TransferScreen, VerifyScreen"
  ```

---

## Task 6: Extract `ConfigScreen` and `EditMachineScreen`

**Files:**
- Create: `src/dcist_launch_system/tui/screens/config.py`
- Modify: `src/dcist_launch_system/tui/screens/__init__.py`
- Modify: `src/dcist_launch_system/tui/__init__.py`

`EditMachineScreen` (lines 1123–1193) and `ConfigScreen` (lines 1195–1374) use: `runtime_config`, `topo`, `topology_path`, `_deleted_machines`, `active_network`.

`ConfigScreen` saves back to `topology_path` on `F5` — needs `ctx.topology_path` and mutates `ctx.runtime_config` and `ctx._deleted_machines`.

- [ ] **Step 1: Create `screens/config.py`** with both `EditMachineScreen(ctx: TuiContext, machine_name: str)` and `ConfigScreen(ctx: TuiContext)`.

  Key mutations to update:
  - `runtime_config[name] = {...}` → `self.ctx.runtime_config[name] = {...}`
  - `_deleted_machines.add(name)` → `self.ctx._deleted_machines.add(name)`
  - Save logic uses `self.ctx.topology_path`

- [ ] **Step 2: Update `screens/__init__.py`** to export `ConfigScreen`, `EditMachineScreen`.

- [ ] **Step 3: Update `tui/__init__.py`** — replace class definitions with imports, pass `ctx` to `ConfigScreen()` in `action_show_config`.

- [ ] **Step 4: Verify**

  ```bash
  python bin/fleet-adt4 --help
  python -c "from dcist_launch_system.tui.screens import ConfigScreen; print('OK')"
  ```

- [ ] **Step 5: Commit**

  ```bash
  git add src/dcist_launch_system/tui/screens/config.py src/dcist_launch_system/tui/screens/__init__.py src/dcist_launch_system/tui/__init__.py
  git commit -m "fleet-adt4 tui: extract ConfigScreen + EditMachineScreen"
  ```

---

## Task 7: Extract `LaunchScreen`

**Files:**
- Create: `src/dcist_launch_system/tui/screens/launch.py`
- Modify: `src/dcist_launch_system/tui/screens/__init__.py`
- Modify: `src/dcist_launch_system/tui/__init__.py`

`LaunchScreen` (lines 1378–2029) is the most complex screen. Uses: `runtime_config`, `topo`, `active_network`, `output_root`, `_local_ips`. Also calls `_check_sessions_and_prompt()` which is a local helper inside `_launch_tui()`.

`_check_sessions_and_prompt()` (lines 667–708) uses: `runtime_config`. Move it to `context.py` as a module-level function taking `ctx` as first argument.

- [ ] **Step 1: Move `_check_sessions_and_prompt()` to `context.py`**

  ```python
  # In context.py, add after TuiContext:
  async def check_sessions_and_prompt(ctx: TuiContext, app, robot_sessions: list[dict]) -> bool:
      """Check for conflicting tmux sessions and prompt for kill.

      Returns True if safe to proceed, False if user cancelled.
      """
      # ... copy exact body from _check_sessions_and_prompt(), replacing:
      # - runtime_config → ctx.runtime_config
      # - ConfirmScreen must be imported here
      from dcist_launch_system.tui.screens.confirm import ConfirmScreen
      # ... rest of function
  ```

- [ ] **Step 2: Create `screens/launch.py`** with `LaunchScreen(ctx: TuiContext, mode: str | None = None)`.

  Replace all closure captures with `self.ctx.*`.
  Replace `_check_sessions_and_prompt(...)` call with `await check_sessions_and_prompt(self.ctx, self.app, ...)` (import from `context`).

- [ ] **Step 3: Update `screens/__init__.py`** to export `LaunchScreen`.

- [ ] **Step 4: Update `tui/__init__.py`** — replace `LaunchScreen` class definition with import, pass `ctx` to `LaunchScreen(ctx, mode=mode)` in `action_show_launch`.

- [ ] **Step 5: Verify**

  ```bash
  python bin/fleet-adt4 --help
  python -c "from dcist_launch_system.tui.screens import LaunchScreen; print('OK')"
  ```

- [ ] **Step 6: Commit**

  ```bash
  git add src/dcist_launch_system/tui/screens/launch.py src/dcist_launch_system/tui/context.py src/dcist_launch_system/tui/screens/__init__.py src/dcist_launch_system/tui/__init__.py
  git commit -m "fleet-adt4 tui: extract LaunchScreen, move check_sessions_and_prompt"
  ```

---

## Task 8: Extract `BandwidthSelectScreen` and `MonitorScreen`

**Files:**
- Create: `src/dcist_launch_system/tui/screens/bandwidth.py`
- Create: `src/dcist_launch_system/tui/screens/monitor.py`
- Modify: `src/dcist_launch_system/tui/screens/__init__.py`
- Modify: `src/dcist_launch_system/tui/__init__.py`

`BandwidthSelectScreen` (lines 2033–2134) uses: `runtime_config`.
`MonitorScreen` (lines 2138–2749) uses: `runtime_config`, `topo`, `active_network`, `_local_ips`. Also references `_start_fleet_zenohd()` and `_stop_fleet_zenohd()` — these stay in `app.py` (Task 9) and get passed via `ctx._fleet_zenoh` plus callbacks.

**Note on zenohd callbacks:** `MonitorScreen` calls `_start_fleet_zenohd()` on `F2`. Since `_start_fleet_zenohd` is a closure function (uses `topo`, `active_network`, etc.) it will be extracted to `app.py` and stored as `ctx.start_fleet_zenohd` (a callable). For now (this task), keep `_start_fleet_zenohd` in `__init__.py` and pass it as an argument to `MonitorScreen`:

```python
MonitorScreen(ctx, start_zenohd=_start_fleet_zenohd, stop_zenohd=_stop_fleet_zenohd)
```

- [ ] **Step 1: Create `screens/bandwidth.py`** with `BandwidthSelectScreen(ctx: TuiContext, machines: list[dict])`.

- [ ] **Step 2: Create `screens/monitor.py`** with `MonitorScreen(ctx: TuiContext, start_zenohd, stop_zenohd)`.

  Replace all closure captures with `self.ctx.*`.
  Store callbacks as `self._start_zenohd = start_zenohd` and `self._stop_zenohd = stop_zenohd`.

- [ ] **Step 3: Update `screens/__init__.py`** to export `BandwidthSelectScreen`, `MonitorScreen`.

- [ ] **Step 4: Update `tui/__init__.py`** — replace class definitions with imports, update `action_show_monitor` to:

  ```python
  def action_show_monitor(self):
      self.push_screen(MonitorScreen(ctx, _start_fleet_zenohd, _stop_fleet_zenohd))
  ```

- [ ] **Step 5: Verify**

  ```bash
  python bin/fleet-adt4 --help
  python -c "from dcist_launch_system.tui.screens import MonitorScreen; print('OK')"
  ```

- [ ] **Step 6: Commit**

  ```bash
  git add src/dcist_launch_system/tui/screens/bandwidth.py src/dcist_launch_system/tui/screens/monitor.py src/dcist_launch_system/tui/screens/__init__.py src/dcist_launch_system/tui/__init__.py
  git commit -m "fleet-adt4 tui: extract BandwidthSelectScreen, MonitorScreen"
  ```

---

## Task 9: Extract `ZenohScreen` and finalize `app.py`

**Files:**
- Create: `src/dcist_launch_system/tui/screens/zenoh_screen.py`
- Create: `src/dcist_launch_system/tui/app.py`
- Modify: `src/dcist_launch_system/tui/screens/__init__.py`
- Modify: `src/dcist_launch_system/tui/__init__.py`

`ZenohScreen` (lines 2753–2874) uses: `runtime_config`, `topo`, `active_network`.
`FleetApp` (lines 2878–2993) uses all context. `_start/_stop_fleet_zenohd` (lines 3001–3100) use all context.

- [ ] **Step 1: Create `screens/zenoh_screen.py`** with `ZenohScreen(ctx: TuiContext)`.

- [ ] **Step 2: Create `tui/app.py`**

  Extract from `tui/__init__.py`:
  - `FleetApp` class
  - `_start_fleet_zenohd(ctx: TuiContext) -> tuple[bool, str]`
  - `_stop_fleet_zenohd(ctx: TuiContext) -> None`
  - `_launch_tui(network, topology_path, output_root)` function

  ```python
  # src/dcist_launch_system/tui/app.py
  """FleetApp — main Textual application + zenohd lifecycle."""
  from __future__ import annotations

  import os
  import shlex
  import shutil
  import subprocess
  import sys
  import threading
  import time

  from textual.app import App, ComposeResult
  from textual.binding import Binding
  from textual.containers import Vertical
  from textual.widgets import DataTable, Footer, Header, Label, RichLog, Rule

  from dcist_launch_system.fleet_helpers import (
      detect_network, filter_reachable, generate_zenoh_endpoints,
      get_local_ips, load_topology, patch_zenoh_config_local, resolve_machines,
  )
  from dcist_launch_system.tui.context import TuiContext
  from dcist_launch_system.tui.screens import (
      MapsScreen, LaunchScreen, ConfigScreen, MonitorScreen, ZenohScreen,
  )


  def _stop_fleet_zenohd(ctx: TuiContext) -> None:
      """Terminate fleet-managed zenohd."""
      proc = ctx._fleet_zenoh["proc"]
      if proc and proc.poll() is None:
          proc.terminate()
          try:
              proc.wait(timeout=5)
          except subprocess.TimeoutExpired:
              proc.kill()
              proc.wait(timeout=2)
          time.sleep(1)
      ctx._fleet_zenoh["proc"] = None


  def _start_fleet_zenohd(ctx: TuiContext) -> tuple[bool, str]:
      """Start (or restart) fleet-managed zenohd. Returns (ok, message)."""
      if not shutil.which("ros2"):
          return False, "ros2 not found on PATH — source your ROS workspace first"
      if ctx._fleet_zenoh["proc"] and ctx._fleet_zenoh["proc"].poll() is None:
          return True, "already running"
      try:
          # ... copy exact body from closure version, replacing:
          # topo → ctx.topo
          # active_network → ctx.active_network
          # runtime_config → ctx.runtime_config
          # _local_ips → ctx._local_ips
          # _fleet_zenoh → ctx._fleet_zenoh
          pass
      except Exception as e:
          return False, f"error: {e}"


  class FleetApp(App):
      # ... copy exact body, replacing all closure var references with ctx.*
      pass


  def _launch_tui(network: str | None, topology_path: str | None, output_root: str) -> None:
      """Build TuiContext and run FleetApp."""
      import click
      try:
          # confirm textual is available
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
  ```

- [ ] **Step 3: Update `screens/__init__.py`** to export `ZenohScreen`.

- [ ] **Step 4: Update `tui/__init__.py`** to import `_launch_tui` from `app.py` instead of defining it:

  ```python
  from dcist_launch_system.tui.app import _launch_tui
  ```

  The `cli` click group (and all subcommands) stay in `tui/__init__.py`. They still call `_launch_tui(network, topology, output_root)`.

- [ ] **Step 5: Verify**

  ```bash
  python bin/fleet-adt4 --help
  python -c "from dcist_launch_system.tui.app import FleetApp, _launch_tui; print('OK')"
  ```

- [ ] **Step 6: Commit**

  ```bash
  git add src/dcist_launch_system/tui/app.py src/dcist_launch_system/tui/screens/zenoh_screen.py src/dcist_launch_system/tui/screens/__init__.py src/dcist_launch_system/tui/__init__.py
  git commit -m "fleet-adt4 tui: extract ZenohScreen, FleetApp, app.py (option 2 complete)"
  ```

---

## Task 10: Automated tests

**Files:**
- Create: `tests/tui/test_import.py`
- Create: `tests/tui/test_screens.py`
- Modify: `tests/tui/conftest.py` (update `ctx` fixture now that `TuiContext` is in `context.py`)

### 10a — Import smoke tests

- [ ] **Step 1: Create `tests/tui/test_import.py`**

  These tests verify the module structure is correct and all public names are importable. No TUI is launched.

  ```python
  # tests/tui/test_import.py
  """Smoke tests: module structure and importability."""


  def test_cli_importable():
      from dcist_launch_system.tui import cli
      assert callable(cli)


  def test_context_importable():
      from dcist_launch_system.tui.context import TuiContext
      assert TuiContext is not None


  def test_app_importable():
      from dcist_launch_system.tui.app import FleetApp, _launch_tui
      assert FleetApp is not None
      assert callable(_launch_tui)


  def test_all_screens_importable():
      from dcist_launch_system.tui.screens import (
          ConfirmScreen,
          MapsScreen,
          PriorMapSelector,
          TransferScreen,
          VerifyScreen,
          ConfigScreen,
          EditMachineScreen,
          LaunchScreen,
          BandwidthSelectScreen,
          MonitorScreen,
          ZenohScreen,
      )
      screens = [
          ConfirmScreen, MapsScreen, PriorMapSelector, TransferScreen,
          VerifyScreen, ConfigScreen, EditMachineScreen, LaunchScreen,
          BandwidthSelectScreen, MonitorScreen, ZenohScreen,
      ]
      for s in screens:
          assert s is not None, f"{s.__name__} is None"


  def test_maybe_reexec_importable():
      from dcist_launch_system.tui import _maybe_reexec
      assert callable(_maybe_reexec)


  def test_no_side_effects_on_import():
      """Importing the package must NOT launch TUI, start processes, or call sys.exit."""
      import importlib
      # If this doesn't raise or hang, the module-level code is safe
      importlib.import_module("dcist_launch_system.tui")
  ```

- [ ] **Step 2: Run import tests**

  ```bash
  cd /home/rrg/dcist_ws/src/awesome_dcist_t4/dcist_launch_system
  python -m pytest tests/tui/test_import.py -v
  ```

  Expected: all 6 tests pass.

### 10b — CLI tests

- [ ] **Step 3: Add CLI tests to `test_import.py`**

  ```python
  # Add to tests/tui/test_import.py

  from click.testing import CliRunner


  def test_cli_help():
      from dcist_launch_system.tui import cli
      runner = CliRunner()
      result = runner.invoke(cli, ["--help"])
      assert result.exit_code == 0
      assert "fleet" in result.output.lower() or "adt4" in result.output.lower()


  def test_status_help():
      from dcist_launch_system.tui import cli
      runner = CliRunner()
      result = runner.invoke(cli, ["status", "--help"])
      assert result.exit_code == 0
      assert "--role" in result.output


  def test_maps_help():
      from dcist_launch_system.tui import cli
      runner = CliRunner()
      result = runner.invoke(cli, ["maps", "--help"])
      assert result.exit_code == 0
      assert "--machine" in result.output


  def test_transfer_help():
      from dcist_launch_system.tui import cli
      runner = CliRunner()
      result = runner.invoke(cli, ["transfer", "--help"])
      assert result.exit_code == 0
      assert "--from" in result.output


  def test_verify_help():
      from dcist_launch_system.tui import cli
      runner = CliRunner()
      result = runner.invoke(cli, ["verify", "--help"])
      assert result.exit_code == 0


  def test_monitor_help():
      from dcist_launch_system.tui import cli
      runner = CliRunner()
      result = runner.invoke(cli, ["monitor", "--help"])
      assert result.exit_code == 0


  def test_zenoh_help():
      from dcist_launch_system.tui import cli
      runner = CliRunner()
      result = runner.invoke(cli, ["zenoh", "--help"])
      assert result.exit_code == 0
  ```

- [ ] **Step 4: Run CLI tests**

  ```bash
  python -m pytest tests/tui/test_import.py -v
  ```

  Expected: all 13 tests pass.

### 10c — Textual screen tests

- [ ] **Step 5: Create `tests/tui/test_screens.py`**

  These tests use `App.run_test()` (Textual's testing API) to mount each screen and verify its DOM composes without errors.

  ```python
  # tests/tui/test_screens.py
  """Textual screen compose/mount tests.

  Each test mounts the screen inside a minimal App and verifies:
  - compose() runs without raising
  - Expected widget IDs are present
  """
  import pytest
  import pytest_asyncio

  pytestmark = pytest.mark.asyncio


  async def test_confirm_screen_composes():
      from textual.app import App, ComposeResult
      from dcist_launch_system.tui.screens.confirm import ConfirmScreen

      class TestApp(App):
          def compose(self) -> ComposeResult:
              return iter([])

          def on_mount(self):
              self.push_screen(ConfirmScreen("Are you sure?"))

      async with TestApp().run_test() as pilot:
          # Modal is pushed on mount — check it rendered
          assert pilot.app.screen is not None
          screen = pilot.app.screen
          assert screen.query("Label")


  async def test_confirm_screen_yes():
      from textual.app import App, ComposeResult
      from dcist_launch_system.tui.screens.confirm import ConfirmScreen

      result_holder = {}

      class TestApp(App):
          def compose(self) -> ComposeResult:
              return iter([])

          async def on_mount(self):
              result = await self.push_screen_wait(ConfirmScreen("Proceed?"))
              result_holder["value"] = result

      async with TestApp().run_test() as pilot:
          await pilot.press("y")
          await pilot.pause()
          assert result_holder.get("value") is True


  async def test_confirm_screen_no():
      from textual.app import App, ComposeResult
      from dcist_launch_system.tui.screens.confirm import ConfirmScreen

      result_holder = {}

      class TestApp(App):
          def compose(self) -> ComposeResult:
              return iter([])

          async def on_mount(self):
              result = await self.push_screen_wait(ConfirmScreen("Proceed?"))
              result_holder["value"] = result

      async with TestApp().run_test() as pilot:
          await pilot.press("n")
          await pilot.pause()
          assert result_holder.get("value") is False


  async def test_maps_screen_composes(ctx):
      from textual.app import App, ComposeResult
      from textual.widgets import Footer
      from dcist_launch_system.tui.screens.maps import MapsScreen

      class TestApp(App):
          def compose(self) -> ComposeResult:
              return iter([])

          def on_mount(self):
              self.push_screen(MapsScreen(ctx))

      async with TestApp().run_test() as pilot:
          assert pilot.app.screen is not None


  async def test_config_screen_composes(ctx):
      from textual.app import App, ComposeResult
      from dcist_launch_system.tui.screens.config import ConfigScreen

      class TestApp(App):
          def compose(self) -> ComposeResult:
              return iter([])

          def on_mount(self):
              self.push_screen(ConfigScreen(ctx))

      async with TestApp().run_test() as pilot:
          assert pilot.app.screen is not None


  async def test_zenoh_screen_composes(ctx):
      from textual.app import App, ComposeResult
      from dcist_launch_system.tui.screens.zenoh_screen import ZenohScreen

      class TestApp(App):
          def compose(self) -> ComposeResult:
              return iter([])

          def on_mount(self):
              self.push_screen(ZenohScreen(ctx))

      async with TestApp().run_test() as pilot:
          assert pilot.app.screen is not None


  async def test_fleet_app_composes(ctx):
      """FleetApp's main screen (not a pushed screen) should compose without error."""
      from dcist_launch_system.tui.app import FleetApp

      async with FleetApp(ctx).run_test() as pilot:
          # Check the status table and log are present
          from textual.widgets import DataTable, RichLog
          assert pilot.app.query_one("#status_table", DataTable)
          assert pilot.app.query_one("#log", RichLog)
  ```

- [ ] **Step 6: Install `pytest-asyncio` if needed**

  ```bash
  pip install pytest-asyncio
  ```

- [ ] **Step 7: Add `asyncio_mode = "auto"` to pytest config**

  Check if `pyproject.toml` or `setup.cfg` exists:
  ```bash
  grep -r "asyncio_mode\|pytest.ini\|pyproject" /home/rrg/dcist_ws/src/awesome_dcist_t4/dcist_launch_system/ --include="*.toml" --include="*.cfg" --include="*.ini" -l
  ```

  If `pyproject.toml` exists, add:
  ```toml
  [tool.pytest.ini_options]
  asyncio_mode = "auto"
  ```

- [ ] **Step 8: Run screen tests**

  ```bash
  cd /home/rrg/dcist_ws/src/awesome_dcist_t4/dcist_launch_system
  python -m pytest tests/tui/test_screens.py -v
  ```

  Expected: all 7 screen tests pass.

- [ ] **Step 9: Run full test suite**

  ```bash
  python -m pytest tests/ -v
  ```

  Expected: all tests pass (existing + new).

- [ ] **Step 10: Commit**

  ```bash
  git add tests/tui/
  git commit -m "fleet-adt4 tui: add automated tests (import, CLI, screen compose)"
  ```

---

## Task 11: Code review with superpowers

- [ ] **Step 1: Invoke code review**

  Use the `superpowers:requesting-code-review` skill (or `superpowers:code-reviewer`) to review the full implementation against:
  - PR #313 requirements (modularization comment #1)
  - Absence of regressions in existing functionality
  - Module boundary correctness (no circular imports, clean `__init__.py` re-exports)
  - Test coverage adequacy

---

## Self-Review

**Spec coverage check:**
- [x] Option 1: thin entry point — Task 1
- [x] Option 2: split by screen with TuiContext — Tasks 3–9
- [x] Automated tests — Task 10
- [x] Code review — Task 11
- [x] `_maybe_reexec` wrapping — Task 1 Step 1
- [x] `_check_sessions_and_prompt` migration — Task 7 Step 1
- [x] zenohd callbacks passed to MonitorScreen — Task 8

**Placeholder scan:** Tasks 4–8 use the phrase "copy exact body" rather than showing code. This is intentional — the bodies are 100–650 lines each and reproducing them verbatim here would make the plan unreadable. The instruction is unambiguous: copy from the source, then replace closure variable references per the table at the top.

**Type consistency:** `TuiContext` fields are defined in Task 2 Step 1 and used consistently in Tasks 3–9. `check_sessions_and_prompt(ctx, app, robot_sessions)` is defined in Task 7 Step 1 and called in `LaunchScreen`.
