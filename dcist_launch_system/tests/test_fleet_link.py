"""Fleet link mode: router endpoints, fleetlink endpoints, fleet_link_env.

Addresses are documentation ranges (RFC 5737) and loopback only.
"""

import copy
import pathlib

import pytest
import yaml

from dcist_launch_system.fleet_helpers import (
    FleetLinkError,
    fleet_link_env,
    fleet_link_mode,
    fleetlink_summary,
    generate_fleetlink_endpoints,
    generate_zenoh_endpoints,
    load_topology,
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
    for name in t["machines"]:
        for net in t["networks"]:
            assert generate_zenoh_endpoints(t, net, name) == []


# --- fleet_link_env -------------------------------------------------------------


def test_fleet_link_env_router_mode():
    assert fleet_link_env("alpha", topo("router")) == {
        "ADT4_FLEETLINK_MODE": "router",
        "ADT4_FLEETLINK_CONNECT": "tcp/127.0.0.1:7447",
        "ADT4_FLEETLINK_PORT": "7450",
    }
    t = topo()
    del t["fleet_defaults"]["fleet_link"]  # no fleet_link: router mode
    assert fleet_link_env("nosuch", t)["ADT4_FLEETLINK_MODE"] == "router"


def test_fleet_link_env_fleetlink_robot_and_base():
    t = topo("fleetlink", fleetlink_port=17450)
    env = fleet_link_env("alpha", t)
    assert env == {
        "ADT4_FLEETLINK_MODE": "fleetlink",
        "ADT4_FLEETLINK_CONNECT": "tcp/198.51.100.100:17450,tcp/192.0.2.3:17450",
        "ADT4_FLEETLINK_PORT": "17450",
    }
    base = fleet_link_env("gamma", t)
    assert base["ADT4_FLEETLINK_CONNECT"] == "tcp/127.0.0.1:17450"


def test_fleet_link_env_mode_override():
    env = fleet_link_env("alpha", topo("fleetlink"), mode="router")
    assert env["ADT4_FLEETLINK_CONNECT"] == "tcp/127.0.0.1:7447"
    env = fleet_link_env("alpha", topo("router"), mode="fleetlink")
    assert env["ADT4_FLEETLINK_CONNECT"].startswith("tcp/198.51.100.100:7450")


@pytest.mark.parametrize(
    "machine, t, mode, match",
    [
        ("alpha", topo("router"), "bogus", "unknown fleet link mode 'bogus'"),
        ("alpha", topo("radio"), None, "unknown fleet link mode 'radio'"),
        ("nosuch", topo("fleetlink"), None, "'nosuch' is not a machine"),
        (
            "alpha",
            topo("fleetlink", fleetlink_networks=["penn_wifi"]),
            None,
            "'gamma' has no address on zenoh.fleetlink_networks",
        ),
    ],
)
def test_fleet_link_env_errors(machine, t, mode, match):
    with pytest.raises(FleetLinkError, match=match):
        fleet_link_env(machine, t, mode)


def test_fleet_link_env_real_topology(topology_path):
    env = fleet_link_env("willow", load_topology(topology_path))
    assert env["ADT4_FLEETLINK_CONNECT"] == "tcp/127.0.0.1:7450"


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
