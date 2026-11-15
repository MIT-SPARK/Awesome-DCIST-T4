"""Textual screen compose/mount tests.

Each test mounts the relevant screen in a minimal App and verifies it renders
without raising exceptions. Requires textual[dev] to be installed.
"""

from __future__ import annotations

import asyncio

import pytest

# ---------------------------------------------------------------------------
# ConfirmScreen
# ---------------------------------------------------------------------------


@pytest.mark.asyncio
async def test_confirm_screen_composes():
    from textual.app import App, ComposeResult

    from dcist_launch_system.tui.screens.confirm import ConfirmScreen

    class TestApp(App):
        def compose(self) -> ComposeResult:
            return iter([])

        def on_mount(self):
            self.push_screen(ConfirmScreen("Are you sure?"))

    async with TestApp().run_test() as pilot:
        assert pilot.app.screen is not None


@pytest.mark.asyncio
async def test_confirm_screen_yes_dismisses():
    """Pressing 'y' should dismiss the confirm screen."""
    from textual.app import App, ComposeResult

    from dcist_launch_system.tui.screens.confirm import ConfirmScreen

    class TestApp(App):
        def compose(self) -> ComposeResult:
            return iter([])

        def on_mount(self):
            self.push_screen(ConfirmScreen("Proceed?"))

    async with TestApp().run_test() as pilot:
        # The ConfirmScreen should be the active screen
        assert isinstance(pilot.app.screen, ConfirmScreen)
        await pilot.press("y")
        await pilot.pause(0.1)
        # After dismissal, the base (empty) App screen is restored
        assert not isinstance(pilot.app.screen, ConfirmScreen)


@pytest.mark.asyncio
async def test_confirm_screen_no_dismisses():
    """Pressing 'n' should dismiss the confirm screen."""
    from textual.app import App, ComposeResult

    from dcist_launch_system.tui.screens.confirm import ConfirmScreen

    class TestApp(App):
        def compose(self) -> ComposeResult:
            return iter([])

        def on_mount(self):
            self.push_screen(ConfirmScreen("Proceed?"))

    async with TestApp().run_test() as pilot:
        assert isinstance(pilot.app.screen, ConfirmScreen)
        await pilot.press("n")
        await pilot.pause(0.1)
        assert not isinstance(pilot.app.screen, ConfirmScreen)


# ---------------------------------------------------------------------------
# MapsScreen
# ---------------------------------------------------------------------------


@pytest.mark.asyncio
async def test_maps_screen_composes(ctx):
    from textual.app import App, ComposeResult

    from dcist_launch_system.tui.screens.maps import MapsScreen

    class TestApp(App):
        def compose(self) -> ComposeResult:
            return iter([])

        def on_mount(self):
            self.push_screen(MapsScreen(ctx))

    async with TestApp().run_test() as pilot:
        assert pilot.app.screen is not None


# ---------------------------------------------------------------------------
# ConfigScreen
# ---------------------------------------------------------------------------


@pytest.mark.asyncio
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


# ---------------------------------------------------------------------------
# ZenohScreen
# ---------------------------------------------------------------------------


@pytest.mark.asyncio
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


# ---------------------------------------------------------------------------
# MonitorScreen
# ---------------------------------------------------------------------------


@pytest.mark.asyncio
async def test_monitor_screen_composes(ctx):
    from textual.app import App, ComposeResult

    from dcist_launch_system.tui.screens.monitor import MonitorScreen

    class TestApp(App):
        def compose(self) -> ComposeResult:
            return iter([])

        def on_mount(self):
            self.push_screen(MonitorScreen(ctx))

    async with TestApp().run_test() as pilot:
        assert pilot.app.screen is not None


# ---------------------------------------------------------------------------
# FleetApp
# ---------------------------------------------------------------------------


@pytest.mark.asyncio
async def test_fleet_app_composes(ctx):
    """FleetApp main screen should compose and render its core widgets."""
    from textual.widgets import DataTable, RichLog

    from dcist_launch_system.tui.app import FleetApp

    async with FleetApp(ctx).run_test() as pilot:
        assert pilot.app.query_one("#status_table", DataTable) is not None
        assert pilot.app.query_one("#log", RichLog) is not None


# ---------------------------------------------------------------------------
# ConfigScreen save: addresses go to the overlay, never to topology.yaml
# ---------------------------------------------------------------------------


async def _save_config(ctx):
    from textual.app import App, ComposeResult

    from dcist_launch_system.tui.screens.config import ConfigScreen

    screen = ConfigScreen(ctx)

    class TestApp(App):
        def compose(self) -> ComposeResult:
            return iter([])

        def on_mount(self):
            self.push_screen(screen)

    async with TestApp().run_test() as pilot:
        await pilot.pause()
        screen.action_save_action()
        await pilot.pause()
        log = screen.query_one("#config_log")
        return "\n".join(line.text for line in log.lines)


def _has_private_keys(topo):
    if any("subnet" in (n or {}) for n in topo.get("networks", {}).values()):
        return True
    if any("addresses" in m for m in topo.get("machines", {}).values()):
        return True
    radios = topo.get("silvus_radios", {})
    return "mgmt_subnet" in radios or "radios" in radios


def test_config_save_splits_addresses_into_overlay(ctx, tmp_path):
    """Sync (asyncio.run) so it runs without pytest-asyncio."""
    import copy
    import os

    import yaml

    from dcist_launch_system.fleet_helpers import topology_merge

    merged = copy.deepcopy(ctx.topo)
    ctx.topology_path = str(tmp_path / "topology.yaml")
    ctx.addresses_path = str(tmp_path / "fleet_addresses.yaml")
    asyncio.run(_save_config(ctx))

    public = yaml.safe_load(open(ctx.topology_path))
    private = yaml.safe_load(open(ctx.addresses_path))
    assert not _has_private_keys(public)
    assert "203.0.113" not in open(ctx.topology_path).read()
    assert "198.51.100" not in open(ctx.topology_path).read()
    assert private["machines"]["alpha"]["addresses"]["silvus"] == "203.0.113.10"
    assert topology_merge.deep_merge(public, private) == merged
    assert os.stat(ctx.addresses_path).st_mode & 0o777 == 0o600


def test_config_save_without_overlay_writes_no_addresses(ctx, tmp_path):
    import yaml

    ctx.topology_path = str(tmp_path / "topology.yaml")
    ctx.addresses_path = None
    text = asyncio.run(_save_config(ctx))

    public = yaml.safe_load(open(ctx.topology_path))
    assert not _has_private_keys(public)
    assert public["machines"]["alpha"]["role"] == "robot"
    assert list(public["networks"]) == ["mit_wifi", "silvus"]
    assert "Addresses NOT saved" in text
    assert list(tmp_path.iterdir()) == [tmp_path / "topology.yaml"]
