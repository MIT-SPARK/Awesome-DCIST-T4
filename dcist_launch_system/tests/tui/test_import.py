"""Smoke tests: module structure and importability."""

from click.testing import CliRunner


def test_cli_importable():
    from dcist_launch_system.tui import cli

    assert callable(cli)


def test_launch_tui_importable():
    from dcist_launch_system.tui.app import _launch_tui

    assert callable(_launch_tui)


def test_no_side_effects_on_import():
    """Importing the package must NOT launch TUI, start processes, or call sys.exit."""
    import importlib

    importlib.import_module("dcist_launch_system.tui")


def test_context_importable():
    from dcist_launch_system.tui.context import TuiContext

    assert TuiContext is not None


def test_app_importable():
    from dcist_launch_system.tui.app import FleetApp, _launch_tui

    assert FleetApp is not None
    assert callable(_launch_tui)


def test_all_screens_importable():
    from dcist_launch_system.tui.screens import (
        BandwidthSelectScreen,
        ConfigScreen,
        ConfirmScreen,
        EditMachineScreen,
        LaunchScreen,
        MapsScreen,
        MonitorScreen,
        PriorMapSelector,
        TransferScreen,
        VerifyScreen,
        ZenohScreen,
    )

    screens = [
        BandwidthSelectScreen,
        ConfigScreen,
        ConfirmScreen,
        EditMachineScreen,
        LaunchScreen,
        MapsScreen,
        MonitorScreen,
        PriorMapSelector,
        TransferScreen,
        VerifyScreen,
        ZenohScreen,
    ]
    for s in screens:
        assert s is not None, f"{s} is None"


def test_cli_help():
    from dcist_launch_system.tui import cli

    runner = CliRunner()
    result = runner.invoke(cli, ["--help"])
    assert result.exit_code == 0
    assert "adt4" in result.output.lower() or "fleet" in result.output.lower()


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
