"""tf_relay heartbeat status (pure function; ROS modules are stubbed)."""

import importlib
import sys
import types

import pytest


@pytest.fixture(scope="module")
def tf_relay():
    stubs = {}
    for name in [
        "rclpy",
        "rclpy.node",
        "rclpy.qos",
        "ros_system_monitor_msgs",
        "ros_system_monitor_msgs.msg",
        "tf2_msgs",
        "tf2_msgs.msg",
    ]:
        if name not in sys.modules:
            stubs[name] = sys.modules[name] = types.ModuleType(name)

    def _qos(*args, **kwargs):
        return None

    if "rclpy.node" in stubs:
        stubs["rclpy.node"].Node = object
    if "rclpy.qos" in stubs:
        stubs["rclpy.qos"].QoSProfile = _qos
        stubs["rclpy.qos"].DurabilityPolicy = types.SimpleNamespace(
            TRANSIENT_LOCAL=None
        )
        stubs["rclpy.qos"].ReliabilityPolicy = types.SimpleNamespace(RELIABLE=None)
    if "ros_system_monitor_msgs.msg" in stubs:
        stubs["ros_system_monitor_msgs.msg"].NodeInfoMsg = object
    if "tf2_msgs.msg" in stubs:
        stubs["tf2_msgs.msg"].TFMessage = object

    try:
        module = importlib.import_module("dcist_launch_system.tf_relay_node")
        yield module
    finally:
        sys.modules.pop("dcist_launch_system.tf_relay_node", None)
        for name in stubs:
            sys.modules.pop(name, None)


def test_router_mode_no_robots_warns(tf_relay):
    level, notes = tf_relay.relay_status({}, {}, 10.0, 1.0, 2.0, fleetlink=False)
    assert level == tf_relay.WARNING
    assert "No robots" in notes


def test_router_mode_stale_robot_warns(tf_relay):
    level, notes = tf_relay.relay_status(
        {"hamilton": 0}, {"hamilton": 1.0}, 10.0, 1.0, 2.0, fleetlink=False
    )
    assert level == tf_relay.WARNING
    assert "Stale TF: hamilton (9 s ago)" in notes


def test_router_mode_fresh_robot_nominal(tf_relay):
    level, notes = tf_relay.relay_status(
        {"hamilton": 10}, {"hamilton": 9.5}, 10.0, 1.0, 2.0, fleetlink=False
    )
    assert level == tf_relay.NOMINAL
    assert notes == "Relaying hamilton 10.0 Hz"


def test_fleetlink_no_robots_idle(tf_relay):
    level, notes = tf_relay.relay_status({}, {}, 10.0, 1.0, 2.0, fleetlink=True)
    assert (level, notes) == (tf_relay.NOMINAL, tf_relay.FLEETLINK_IDLE_NOTES)


def test_fleetlink_robot_seen_earlier_is_not_a_warning(tf_relay):
    # hamilton was relayed in router mode, then the base switched to fleetlink
    level, notes = tf_relay.relay_status(
        {"hamilton": 0, "euclid": 0},
        {"hamilton": 1.0},
        100.0,
        1.0,
        2.0,
        fleetlink=True,
    )
    assert (level, notes) == (tf_relay.NOMINAL, tf_relay.FLEETLINK_IDLE_NOTES)


def test_fleetlink_fresh_robot_reports_rate(tf_relay):
    level, notes = tf_relay.relay_status(
        {"hamilton": 5, "euclid": 0},
        {"hamilton": 9.9, "euclid": 1.0},
        10.0,
        1.0,
        2.0,
        fleetlink=True,
    )
    assert level == tf_relay.NOMINAL
    assert notes == "Relaying hamilton 5.0 Hz"
