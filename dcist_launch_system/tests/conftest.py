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
    /// Example: endpoints: ["tcp/10.0.0.1:7447"]
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
