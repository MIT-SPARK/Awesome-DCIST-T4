import pathlib
import pytest


@pytest.fixture
def topology_path():
    """Path to the real topology.yaml."""
    return (
        pathlib.Path(__file__).resolve().parents[1]
        / ".."
        / "network_diagnostics"
        / "config"
        / "topology.yaml"
    )


@pytest.fixture
def sample_topology():
    """Minimal topology dict for unit tests."""
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
                "addresses": {
                    "mit_wifi": "10.29.1.1",
                    "silvus": "192.168.100.10",
                },
            },
            "beta": {
                "role": "robot",
                "platform_id": "topaz",
                "desc": "Test Robot B",
                "addresses": {
                    "mit_wifi": "10.29.1.2",
                },
            },
            "gamma": {
                "role": "base_station",
                "desc": "Test Base Station",
                "addresses": {
                    "mit_wifi": "10.29.1.3",
                    "silvus": "192.168.100.100",
                },
            },
        },
        "ssh": {
            "robot_user": "swarm",
            "base_station_user": "rrg",
        },
    }
