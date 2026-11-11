"""run-adt4: fleet link env precedence and zenoh isolation in fleetlink mode."""

import importlib.machinery
import importlib.util
import pathlib

import pytest
import yaml

PKG = pathlib.Path(__file__).resolve().parents[1]


@pytest.fixture(scope="module")
def run_adt4():
    path = PKG / "bin" / "run-adt4"
    loader = importlib.machinery.SourceFileLoader("run_adt4", str(path))
    spec = importlib.util.spec_from_loader("run_adt4", loader)
    module = importlib.util.module_from_spec(spec)
    loader.exec_module(module)
    return module


def test_zenoh_router_mode_keeps_connect_endpoints(run_adt4):
    window = run_adt4._make_zenoh_window("router")
    assert window["panes"] == ["ros2 run rmw_zenoh_cpp rmw_zenohd"]


def test_zenoh_fleetlink_mode_clears_connect_endpoints(run_adt4):
    window = run_adt4._make_zenoh_window("fleetlink")
    assert window["panes"] == [
        "ZENOH_CONFIG_OVERRIDE='connect/endpoints=[]' ros2 run rmw_zenoh_cpp rmw_zenohd"
    ]


def _fake_fleet(mode_default="fleetlink"):
    def fleet_env(robot_name, mode=None):
        mode = mode or mode_default
        port = 7450
        if mode == "fleetlink":
            return {
                "ADT4_FLEETLINK_MODE": "fleetlink",
                "ADT4_FLEETLINK_CONNECT": f"tcp/192.0.2.3:{port}",
                "ADT4_FLEETLINK_PORT": str(port),
            }
        return {
            "ADT4_FLEETLINK_MODE": "router",
            "ADT4_FLEETLINK_CONNECT": "tcp/127.0.0.1:7447",
            "ADT4_FLEETLINK_PORT": str(port),
        }

    return fleet_env


def test_derive_fills_unset(run_adt4):
    env = {}
    run_adt4._derive_fleet_env(env, "alpha", fleet_env=_fake_fleet())
    assert env["ADT4_FLEETLINK_MODE"] == "fleetlink"
    assert env["ADT4_FLEET_LINK"] == "fleetlink"  # legacy name, same value
    assert env["ADT4_FLEETLINK_CONNECT"] == "tcp/192.0.2.3:7450"
    assert env["ADT4_FLEETLINK_PORT"] == "7450"
    assert "ADT4_FLEETLINK_DERIVED" not in env
    assert set(run_adt4._session_env(env)) == {
        "ADT4_FLEETLINK_MODE",
        "ADT4_FLEETLINK_CONNECT",
        "ADT4_FLEETLINK_PORT",
        "ADT4_FLEET_LINK",
    }


@pytest.mark.parametrize("var", ["ADT4_FLEETLINK_MODE", "ADT4_FLEET_LINK"])
def test_env_mode_wins_and_picks_endpoints(run_adt4, var, capsys):
    env = {var: "router"}
    run_adt4._derive_fleet_env(env, "alpha", fleet_env=_fake_fleet())
    assert env["ADT4_FLEETLINK_MODE"] == env["ADT4_FLEET_LINK"] == "router"
    assert env["ADT4_FLEETLINK_CONNECT"] == "tcp/127.0.0.1:7447"
    assert "overrides topology.yaml's 'fleetlink'" in capsys.readouterr().out


def test_new_mode_var_beats_legacy(run_adt4):
    env = {"ADT4_FLEETLINK_MODE": "fleetlink", "ADT4_FLEET_LINK": "router"}
    run_adt4._derive_fleet_env(env, "alpha", fleet_env=_fake_fleet("router"))
    assert env["ADT4_FLEETLINK_MODE"] == env["ADT4_FLEET_LINK"] == "fleetlink"
    assert env["ADT4_FLEETLINK_CONNECT"] == "tcp/192.0.2.3:7450"


def test_env_mode_matching_topology_is_quiet(run_adt4, capsys):
    # e.g. a nested run-adt4 inheriting its session's mode
    env = {"ADT4_FLEETLINK_MODE": "fleetlink"}
    run_adt4._derive_fleet_env(env, "alpha", fleet_env=_fake_fleet())
    assert env["ADT4_FLEETLINK_MODE"] == "fleetlink"
    assert capsys.readouterr().out == ""


def test_connect_and_port_always_from_topology(run_adt4, capsys):
    env = {
        "ADT4_FLEETLINK_CONNECT": "tcp/198.51.100.7:7450",
        "ADT4_FLEETLINK_PORT": "17450",
    }
    run_adt4._derive_fleet_env(env, "alpha", fleet_env=_fake_fleet())
    assert env["ADT4_FLEETLINK_CONNECT"] == "tcp/192.0.2.3:7450"
    assert env["ADT4_FLEETLINK_PORT"] == "7450"
    assert capsys.readouterr().out == ""


def test_router_fallback_when_fleet_settings_fail(run_adt4, monkeypatch):
    import dcist_launch_system.fleet_env as fe

    def boom(*args, **kwargs):
        raise RuntimeError("no topology")

    monkeypatch.setattr(fe, "fleet_env", boom)
    env = {"ADT4_FLEETLINK_CONNECT": "tcp/198.51.100.7:7450"}
    run_adt4._derive_fleet_env(env, "alpha")
    assert env["ADT4_FLEETLINK_MODE"] == env["ADT4_FLEET_LINK"] == "router"
    assert env["ADT4_FLEETLINK_CONNECT"] == "tcp/127.0.0.1:7447"


def test_fleet_env_falls_back_without_topology(run_adt4, monkeypatch):
    import dcist_launch_system.fleet_env as fe

    def boom(*args, **kwargs):
        raise RuntimeError("no topology")

    monkeypatch.setattr(fe, "fleet_env", boom)
    env = run_adt4._fleet_env("alpha")
    assert env["ADT4_FLEETLINK_MODE"] == "router"


@pytest.mark.parametrize("mode", ["router", "fleetlink"])
def test_session_file_carries_fleet_env(run_adt4, tmp_path, mode):
    session = tmp_path / "s.yaml"
    session.write_text(
        yaml.safe_dump(
            {"session_name": "adt4_system", "windows": [{"window_name": "core"}]}
        )
    )
    env = {"ADT4_FLEETLINK_MODE": mode}
    run_adt4._derive_fleet_env(env, "alpha", fleet_env=_fake_fleet())
    info = run_adt4.SessionInfo(
        "alpha",
        tmp_path,
        fleet_link=env["ADT4_FLEETLINK_MODE"],
        session_env=run_adt4._session_env(env),
    )
    out = tmp_path / "out.yaml"
    with out.open("w") as fout:
        run_adt4._write_session_to_file(session, info, fout)
    contents = yaml.safe_load(out.read_text())
    assert contents["environment"]["ADT4_FLEETLINK_MODE"] == mode
    assert contents["environment"]["ADT4_FLEET_LINK"] == mode
    zenoh = contents["windows"][0]
    assert zenoh["window_name"] == "zenoh"
    assert ("ZENOH_CONFIG_OVERRIDE" in zenoh["panes"][0]) == (mode == "fleetlink")


def test_fleet_env_reads_the_checkout_topology(run_adt4, monkeypatch):
    monkeypatch.delenv("ADT4_TOPOLOGY", raising=False)
    assert run_adt4._topology_path().is_file()
    env = run_adt4._fleet_env("willow")
    assert env["ADT4_FLEETLINK_MODE"] == "fleetlink"  # topology.yaml's default
    assert env["ADT4_FLEETLINK_CONNECT"] == "tcp/127.0.0.1:7450"


@pytest.mark.parametrize(
    "session",
    [
        "spot_regions-indoor_regions",
        "spot_regions_traversability-regions_traversability",
        "spot_bag_regions-indoor_regions",
        "spot_bag_regions_traversability-regions_traversability",
    ],
)
def test_regional_mapping_sessions_have_a_radio_relay(session):
    config = yaml.safe_load(
        (PKG / "tmux" / "autogenerated" / f"{session}.yaml").read_text()
    )
    agents = [w for w in config["windows"] if w["window_name"] == "fleetlink"]
    assert len(agents) == 1
    assert "relay_config:=" in " ".join(agents[0]["panes"])
