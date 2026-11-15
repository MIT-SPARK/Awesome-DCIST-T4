import pytest

from dcist_launch_system.tui.context import TuiContext


@pytest.fixture
def sample_topo():
    return {
        "networks": {
            "mit_wifi": {"subnet": "198.51.100.0/24"},
            "silvus": {"subnet": "203.0.113.0/24"},
        },
        "machines": {
            "alpha": {
                "role": "robot",
                "platform_id": "smaug",
                "desc": "Test Robot A",
                "addresses": {"mit_wifi": "198.51.100.1", "silvus": "203.0.113.10"},
            },
            "beta": {
                "role": "robot",
                "platform_id": "topaz",
                "desc": "Test Robot B",
                "addresses": {"mit_wifi": "198.51.100.2"},
            },
            "gamma": {
                "role": "base_station",
                "desc": "Test Base Station",
                "addresses": {"mit_wifi": "198.51.100.3", "silvus": "203.0.113.100"},
            },
        },
        "ssh": {"robot_user": "swarm", "base_station_user": "rrg"},
        "zenoh": {"port": 7447, "connect_network": "silvus"},
        "fleet_defaults": {
            "spot_platforms": ["smaug", "topaz"],
            "sessions": {
                "mapping": {"spot": "spot-default", "phoenix": "phoenix-default"},
                "relocalize": {
                    "spot": "spot_relocalize-relocalize",
                    "phoenix": "phoenix_relocalize-relocalize",
                },
            },
            "base_station_session": "base_station_heracles-default",
            "output_root": "~/adt4_output",
        },
    }


@pytest.fixture
def ctx(sample_topo):
    """Minimal TuiContext for screen tests."""
    machines = {
        "alpha": {
            "name": "alpha",
            "role": "robot",
            "platform_id": "smaug",
            "ip": "198.51.100.1",
            "online": False,
            "ping": False,
            "ssh": False,
        },
        "beta": {
            "name": "beta",
            "role": "robot",
            "platform_id": "topaz",
            "ip": "198.51.100.2",
            "online": False,
            "ping": False,
            "ssh": False,
        },
        "gamma": {
            "name": "gamma",
            "role": "base_station",
            "platform_id": None,
            "ip": "198.51.100.3",
            "online": False,
            "ping": False,
            "ssh": False,
        },
    }
    return TuiContext(
        topo=sample_topo,
        active_network="mit_wifi",
        topology_path=None,
        output_root="~/adt4_output",
        runtime_config=machines,
    )
