"""Fleet link mode: router endpoints, fleetlink endpoints, fleet_env, the validator.

Addresses are documentation ranges (RFC 5737) and loopback only.
"""

import copy
import pathlib
import shlex
import subprocess
import sys

import pytest
import yaml

from dcist_launch_system import fleet_env as fe
from dcist_launch_system.fleet_helpers import (
    fleet_link_mode,
    fleetlink_summary,
    generate_fleetlink_endpoints,
    generate_zenoh_endpoints,
    load_topology,
    validate_topology,
)

TOPO = {
    "networks": {
        "mit_wifi": {"subnet": "192.0.2.0/24"},
        "silvus": {"subnet": "198.51.100.0/24"},
        "penn_wifi": {"subnet": "203.0.113.0/24"},
    },
    "machines": {
        "alpha": {
            "role": "robot",
            "addresses": {"mit_wifi": "192.0.2.1", "silvus": "198.51.100.10"},
        },
        "beta": {"role": "robot", "addresses": {"mit_wifi": "192.0.2.2"}},
        "gamma": {
            "role": "base_station",
            "addresses": {"mit_wifi": "192.0.2.3", "silvus": "198.51.100.100"},
        },
        "delta": {"role": "base_station", "addresses": {"penn_wifi": "203.0.113.5"}},
    },
    "zenoh": {
        "port": 7447,
        "connect_network": "silvus",
        "fleetlink_port": 7450,
        "fleetlink_networks": ["silvus", "mit_wifi"],
    },
    "fleet_defaults": {"fleet_link": "router", "fleetlink_base": "gamma"},
}


def topo(mode="router", **zenoh):
    t = copy.deepcopy(TOPO)
    t["fleet_defaults"]["fleet_link"] = mode
    t["zenoh"].update(zenoh)
    return t


# --- endpoints -----------------------------------------------------------------


def test_router_mode_endpoints_unchanged():
    t = topo("router")
    assert fleet_link_mode(t) == "router"
    assert generate_zenoh_endpoints(t, "mit_wifi", "gamma") == [
        "tcp/192.0.2.1:7447",
        "tcp/192.0.2.2:7447",
    ]
    assert generate_zenoh_endpoints(t, "silvus", "gamma", robot_filter=["alpha"]) == [
        "tcp/198.51.100.10:7447"
    ]
    assert generate_zenoh_endpoints(t, "mit_wifi", "alpha") == []


@pytest.mark.parametrize("name", ["alpha", "beta", "gamma", "delta"])
def test_fleetlink_mode_no_router_endpoints(name):
    t = topo("fleetlink")
    for net in ("mit_wifi", "silvus", "penn_wifi"):
        assert generate_zenoh_endpoints(t, net, name) == []
        assert generate_zenoh_endpoints(t, net, name, robot_filter=["alpha"]) == []


def test_fleetlink_mode_unknown_machine_still_keyerror():
    # consumers report "no address" on KeyError; keep that contract
    with pytest.raises(KeyError):
        generate_zenoh_endpoints(topo("fleetlink"), "silvus", "nosuch")


def test_missing_or_unknown_mode_is_router():
    assert fleet_link_mode(topo("bogus")) == "router"
    t = topo()
    del t["fleet_defaults"]["fleet_link"]
    assert fleet_link_mode(t) == "router"
    assert fleet_link_mode({}) == "router"
    assert fleet_link_mode(None) == "router"


def test_fleetlink_endpoints_ordered_by_networks():
    t = topo("fleetlink")
    assert generate_fleetlink_endpoints(t) == [
        "tcp/198.51.100.100:7450",
        "tcp/192.0.2.3:7450",
    ]
    t["zenoh"]["fleetlink_networks"] = ["mit_wifi", "silvus"]
    assert generate_fleetlink_endpoints(t) == [
        "tcp/192.0.2.3:7450",
        "tcp/198.51.100.100:7450",
    ]
    # networks the base has no address on are skipped
    t["zenoh"]["fleetlink_networks"] = ["penn_wifi", "mit_wifi"]
    assert generate_fleetlink_endpoints(t) == ["tcp/192.0.2.3:7450"]


def test_fleetlink_endpoints_args_and_defaults():
    t = topo("fleetlink", fleetlink_port=17450)
    assert generate_fleetlink_endpoints(t, base="delta") == []
    assert generate_fleetlink_endpoints(t, networks=["mit_wifi"]) == [
        "tcp/192.0.2.3:17450"
    ]
    assert generate_fleetlink_endpoints(t, base="nosuch") == []
    # no fleetlink_networks: connect_network, then every base network
    del t["zenoh"]["fleetlink_networks"]
    assert generate_fleetlink_endpoints(t) == ["tcp/198.51.100.100:17450"]
    del t["zenoh"]["connect_network"]
    assert generate_fleetlink_endpoints(t, base="delta") == ["tcp/203.0.113.5:17450"]
    # no port: 7450
    del t["zenoh"]["fleetlink_port"]
    assert generate_fleetlink_endpoints(t, networks=["silvus"]) == [
        "tcp/198.51.100.100:7450"
    ]


def test_fleetlink_endpoint_ipv6():
    t = topo("fleetlink")
    t["machines"]["gamma"]["addresses"]["silvus"] = "2001:db8::1"
    assert generate_fleetlink_endpoints(t)[0] == "tcp/[2001:db8::1]:7450"


def test_fleetlink_summary():
    t = topo("fleetlink")
    assert fleetlink_summary(t) == "fleetlink client -> gamma:7450"
    assert fleetlink_summary(t, "robot") == "fleetlink client -> gamma:7450"
    assert "gamma" in fleetlink_summary(t, "base_station")


def test_real_topology_fleetlink_settings(topology_path):
    t = load_topology(topology_path)
    assert fleet_link_mode(t) == "fleetlink"
    assert not [
        p for p in validate_topology(t) if "fleetlink" in p or "fleet_link" in p
    ]
    for name in t["machines"]:
        for net in t["networks"]:
            assert generate_zenoh_endpoints(t, net, name) == []


# --- validator -------------------------------------------------------------------


def test_validate_topology_ok():
    assert validate_topology(topo("fleetlink")) == []
    assert validate_topology(topo("router")) == []


def test_validate_duplicate_address():
    t = topo()
    t["machines"]["beta"]["addresses"]["silvus"] = "198.51.100.10"
    assert validate_topology(t) == [
        "duplicate silvus address 198.51.100.10: alpha and beta"
    ]
    # the same address on different networks is fine
    t["machines"]["beta"]["addresses"] = {"penn_wifi": "198.51.100.10"}
    assert validate_topology(t) == []


def test_validate_fleet_link_and_base():
    t = topo("radio")
    assert any("fleet_link must be one of" in p for p in validate_topology(t))
    t = topo("fleetlink")
    t["fleet_defaults"]["fleetlink_base"] = "alpha"
    assert validate_topology(t) == [
        "fleet_defaults.fleetlink_base 'alpha' is not a base_station"
    ]
    t["fleet_defaults"]["fleetlink_base"] = "nobody"
    assert validate_topology(t) == [
        "fleet_defaults.fleetlink_base 'nobody' is not a machine"
    ]
    del t["fleet_defaults"]["fleetlink_base"]
    assert validate_topology(t) == [
        "fleet_link is 'fleetlink' but fleetlink_base is not set"
    ]


def test_validate_fleetlink_port_and_networks():
    t = topo(fleetlink_port="7450", fleetlink_networks=["silvus", "doodle"])
    problems = validate_topology(t)
    assert "zenoh.fleetlink_port must be an int, got '7450'" in problems
    assert "zenoh.fleetlink_networks: unknown network 'doodle'" in problems


# --- fleet_env -----------------------------------------------------------------


def test_fleet_env_router_mode():
    env = fe.fleet_env("alpha", topo("router"))
    assert env == {
        "ADT4_FLEETLINK_MODE": "router",
        "ADT4_FLEETLINK_CONNECT": "tcp/127.0.0.1:7447",
        "ADT4_FLEETLINK_PORT": "7450",
    }


def test_fleet_env_fleetlink_robot_and_base():
    t = topo("fleetlink")
    env = fe.fleet_env("alpha", t)
    assert env["ADT4_FLEETLINK_MODE"] == "fleetlink"
    assert env["ADT4_FLEETLINK_CONNECT"] == (
        "tcp/198.51.100.100:7450,tcp/192.0.2.3:7450"
    )
    base = fe.fleet_env("gamma", t)
    assert base["ADT4_FLEETLINK_MODE"] == "fleetlink"
    assert base["ADT4_FLEETLINK_CONNECT"] == "tcp/127.0.0.1:7450"
    assert fe.fleet_env("alpha", t, network="mit_wifi")["ADT4_FLEETLINK_CONNECT"] == (
        "tcp/192.0.2.3:7450"
    )


def test_fleet_env_mode_override():
    env = fe.fleet_env("alpha", topo("fleetlink"), mode="router")
    assert env["ADT4_FLEETLINK_MODE"] == "router"
    assert env["ADT4_FLEETLINK_CONNECT"] == "tcp/127.0.0.1:7447"
    env = fe.fleet_env("alpha", topo("router"), mode="fleetlink")
    assert env["ADT4_FLEETLINK_CONNECT"].startswith("tcp/198.51.100.100:7450")
    warnings = []
    env = fe.fleet_env("alpha", topo("router"), mode="bogus", warn=warnings.append)
    assert env["ADT4_FLEETLINK_MODE"] == "router" and warnings


def test_fleet_env_falls_back_to_router():
    warnings = []
    t = topo("fleetlink")
    # unknown machine
    env = fe.fleet_env("nosuch", t, warn=warnings.append)
    assert env["ADT4_FLEETLINK_MODE"] == "router"
    # base unreachable on every fleetlink network
    t2 = topo("fleetlink", fleetlink_networks=["penn_wifi"])
    env = fe.fleet_env("alpha", t2, warn=warnings.append)
    assert env["ADT4_FLEETLINK_MODE"] == "router"
    assert env["ADT4_FLEETLINK_CONNECT"] == "tcp/127.0.0.1:7447"
    # --network the base is not on: keep the full list
    env = fe.fleet_env("alpha", t, network="penn_wifi", warn=warnings.append)
    assert env["ADT4_FLEETLINK_CONNECT"].count(",") == 1
    # garbage topology never raises
    bad = {"machines": 5, "fleet_defaults": {"fleet_link": "fleetlink"}}
    assert fe.fleet_env("alpha", bad, warn=warnings.append)["ADT4_FLEETLINK_MODE"] == (
        "router"
    )
    assert len(warnings) == 4


def test_mode_from_env_reads_legacy_name():
    assert fe.mode_from_env({}) is None
    assert fe.mode_from_env({"ADT4_FLEET_LINK": "router"}) == "router"
    assert fe.mode_from_env({"ADT4_FLEETLINK_MODE": "fleetlink"}) == "fleetlink"
    both = {"ADT4_FLEETLINK_MODE": "fleetlink", "ADT4_FLEET_LINK": "router"}
    assert fe.mode_from_env(both) == "fleetlink"
    assert fe.mode_from_env(
        {"ADT4_FLEETLINK_MODE": "", "ADT4_FLEET_LINK": "router"}
    ) == ("router")


def _cli(*args):
    return subprocess.run(
        [sys.executable, "-m", "dcist_launch_system.fleet_env", *args],
        capture_output=True,
        text=True,
        timeout=30,
    )


def _exports(stdout):
    out = {}
    for line in stdout.splitlines():
        assert line.startswith("export ")
        k, v = line[len("export ") :].split("=", 1)
        out[k] = shlex.split(v)[0]
    return out


def test_fleet_env_cli(tmp_path):
    path = tmp_path / "topology.yaml"
    path.write_text(yaml.safe_dump(topo("fleetlink")))
    r = _cli("alpha", "--topology", str(path))
    assert r.returncode == 0, r.stderr
    assert _exports(r.stdout) == {
        "ADT4_FLEETLINK_MODE": "fleetlink",
        "ADT4_FLEETLINK_CONNECT": "tcp/198.51.100.100:7450,tcp/192.0.2.3:7450",
        "ADT4_FLEETLINK_PORT": "7450",
    }
    r = _cli("alpha", "--topology", str(path), "--network", "mit_wifi")
    assert _exports(r.stdout)["ADT4_FLEETLINK_CONNECT"] == "tcp/192.0.2.3:7450"
    r = _cli("alpha", "--topology", str(path), "--mode", "router")
    assert _exports(r.stdout)["ADT4_FLEETLINK_MODE"] == "router"
    # a missing topology still exits 0 with router mode
    r = _cli("alpha", "--topology", str(tmp_path / "missing.yaml"))
    assert r.returncode == 0
    assert _exports(r.stdout)["ADT4_FLEETLINK_MODE"] == "router"
    assert "router mode" in r.stderr


def test_fleet_env_cli_check(tmp_path):
    path = tmp_path / "topology.yaml"
    path.write_text(yaml.safe_dump(topo("fleetlink")))
    assert _cli("--check", "--topology", str(path)).returncode == 0
    bad = topo("fleetlink")
    bad["fleet_defaults"]["fleetlink_base"] = "alpha"
    path.write_text(yaml.safe_dump(bad))
    r = _cli("--check", "--topology", str(path))
    assert r.returncode == 1 and "not a base_station" in r.stdout


def test_fleet_env_cli_real_topology():
    r = _cli("willow")
    assert r.returncode == 0, r.stderr
    assert set(_exports(r.stdout)) == set(fe.FLEET_VARS)


# --- generated sessions --------------------------------------------------------------

AUTOGEN = pathlib.Path(__file__).resolve().parents[1] / "tmux" / "autogenerated"
RELAY = "/share/dcist_launch_system/config/fleetlink/relay.yaml"
TOPOLOGY = "-p topology:=${ADT4_TOPOLOGY:-$ADT4_WS/src/awesome_dcist_t4/network_diagnostics/config/topology.yaml}"  # noqa: E501


def _window(name, window):
    session = yaml.safe_load((AUTOGEN / f"{name}.yaml").read_text())
    found = [w for w in session["windows"] if w["window_name"] == window]
    return found[0] if found else None


@pytest.mark.parametrize(
    "name",
    [
        "base_station-default",
        "base_station_heracles-default",
        "base_station_repair-default",
        "base_station_heracles_repair-default",
        "base_station_grstaps_heracles_repair-grstaps_repair",
    ],
)
def test_base_sessions_have_fleetlink_base_window(name):
    router, bridge = _window(name, "fleetlink_base")["panes"]
    assert router.startswith('if [ "$ADT4_FLEETLINK_MODE" = fleetlink ]; then ')
    assert "ros2 run fleetlink fleetlink-router ${ADT4_FLEETLINK_PORT:-7450}" in router
    assert bridge.startswith('if [ "$ADT4_FLEETLINK_MODE" = fleetlink ]; then ')
    assert "ros2 run fleetlink fleetlink-bridge --ros-args" in bridge
    assert "-p zenoh_connect:=${ADT4_FLEETLINK_CONNECT:-tcp/127.0.0.1:7450}" in bridge
    assert RELAY in bridge and TOPOLOGY in bridge
    # the bridge writes robot TF where the base reads it: /<base>/tf
    assert (
        "-r /tf:=/$ADT4_ROBOT_NAME/tf -r /tf_static:=/$ADT4_ROBOT_NAME/tf_static"
        in bridge
    )
    assert _window(name, "fleetlink") is None


@pytest.mark.parametrize(
    "name",
    [
        "spot-default",
        "phoenix-default",
        "spot_relocalize-relocalize",
        "phoenix_relocalize-relocalize",
        "spot_bag-default",
        "spot-image_logging",
        "spot_sim_fleetlink-default",
    ],
)
def test_robot_sessions_have_fleetlink_agent(name):
    [agent] = _window(name, "fleetlink")["panes"]
    assert agent.startswith("ros2 run fleetlink fleetlink-agent --ros-args ")
    assert "-p zenoh_connect:=${ADT4_FLEETLINK_CONNECT:-tcp/127.0.0.1:7447}" in agent
    # the topology always (it gives the platform), the relay table only in fleetlink mode
    assert TOPOLOGY in agent
    assert '$([ "$ADT4_FLEETLINK_MODE" = fleetlink ] && echo -p relay_config:=' in agent
    assert RELAY in agent
    assert (
        "-r /tf:=/$ADT4_ROBOT_NAME/tf -r /tf_static:=/$ADT4_ROBOT_NAME/tf_static"
        in agent
    )
    assert _window(name, "fleetlink_base") is None


@pytest.mark.parametrize(
    "name",
    [
        "spot_prior_dsg-prior_dsg",
        "spot_prior_dsg_heracles_repair-prior_dsg",
        "spot_grstaps-grstaps",
    ],
)
def test_single_machine_sims_have_no_fleetlink(name):
    assert _window(name, "fleetlink_base") is None
    assert _window(name, "fleetlink") is None
