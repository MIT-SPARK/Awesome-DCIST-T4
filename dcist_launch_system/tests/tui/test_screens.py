"""Textual screen compose/mount tests.

Each test mounts the relevant screen in a minimal App and verifies it renders
without raising exceptions. Requires textual[dev] to be installed.
"""

from __future__ import annotations

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
