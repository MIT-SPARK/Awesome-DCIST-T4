import pathlib

import pytest

_NETWORK_DIAGNOSTICS = (
    pathlib.Path(__file__).resolve().parents[2] / "network_diagnostics"
)


@pytest.fixture(autouse=True)
def _no_real_fleet_addresses(monkeypatch, tmp_path_factory):
    """Never let a test pick up this machine's real fleet address overlay
    ($ADT4_FLEET_ADDRESSES or ~/.config/adt4/fleet_addresses)."""
    from dcist_launch_system import fleet_helpers

    monkeypatch.delenv("ADT4_FLEET_ADDRESSES", raising=False)
    missing = tmp_path_factory.mktemp("no_overlay") / "fleet_addresses.yaml"
    monkeypatch.setattr(fleet_helpers.topology_merge, "DEFAULT_ADDRESSES_PATH", missing)
    monkeypatch.setattr(fleet_helpers, "_warned_no_overlay", False)


@pytest.fixture
def topology_path():
    """Path to the real (public) topology.yaml."""
    return _NETWORK_DIAGNOSTICS / "config" / "topology.yaml"


@pytest.fixture
def example_addresses_path():
    """Path to fleet_addresses.example.yaml (documentation addresses only)."""
    return _NETWORK_DIAGNOSTICS / "config" / "fleet_addresses.example.yaml"


@pytest.fixture
def topology_merge_script():
    return _NETWORK_DIAGNOSTICS / "scripts" / "topology_merge.py"


@pytest.fixture
def sample_topology():
    """Minimal topology dict for unit tests."""
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
                "addresses": {
                    "mit_wifi": "198.51.100.1",
                    "silvus": "203.0.113.10",
                },
            },
            "beta": {
                "role": "robot",
                "platform_id": "topaz",
                "desc": "Test Robot B",
                "addresses": {
                    "mit_wifi": "198.51.100.2",
                },
            },
            "gamma": {
                "role": "base_station",
                "desc": "Test Base Station",
                "addresses": {
                    "mit_wifi": "198.51.100.3",
                    "silvus": "203.0.113.100",
                },
            },
        },
        "ssh": {
            "robot_user": "swarm",
            "base_station_user": "rrg",
        },
        "zenoh": {
            "port": 7447,
            "connect_network": "silvus",
        },
    }


@pytest.fixture
def rviz_template(tmp_path):
    """Minimal rviz template with topics and TF frames for namespace tests."""
    content = """\
Panels: []
Visualization Manager:
  Displays:
    - Class: rviz_default_plugins/Marker
      Name: HydraGraph
      Value: hydra_visualizer/graph
      Enabled: true
    - Class: rviz_default_plugins/TF
      Frames:
        euclid/base_link:
          Enabled: true
        euclid/body:
          Enabled: true
    - Class: rviz_default_plugins/Image
      Value: frontright/semantic_overlay/image_raw
"""
    f = tmp_path / "dcist.rviz"
    f.write_text(content)
    return f


@pytest.fixture
def zenoh_template(tmp_path):
    """Minimal JSON5 zenoh router template for patching tests."""
    content = """\
{
  mode: "router",
  connect: {
    /// Example: endpoints: ["tcp/192.0.2.1:7447"]
    endpoints: []
  },
  listen: {
    endpoints: [
      "tcp/[::]:7447"
    ]
  },
  transport: {
    link: {
      tx: { sequence_number_resolution: 268435456 }
    }
  }
}
"""
    f = tmp_path / "zenoh_router_template.json5"
    f.write_text(content)
    return f
