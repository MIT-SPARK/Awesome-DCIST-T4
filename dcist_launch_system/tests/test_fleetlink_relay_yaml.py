"""ADT4's fleetlink relay table (dcist_launch_system/fleetlink/relay.yaml).

Needs the fleetlink package importable (``from fleetlink.relay import RelayConfig``):
a built workspace, or the fleetlink submodule on the path (skipped otherwise).
"""

import os
import pathlib
import sys

import pytest

RELAY_YAML = os.path.join(os.path.dirname(__file__), "..", "fleetlink", "relay.yaml")


@pytest.fixture(scope="module")
def relay_config_class():
    try:
        from fleetlink.relay import RelayConfig
    except ImportError:  # not built: use the submodule's sources
        # The hand-written config directory (or submodule root) may have been
        # cached as a namespace package by the failed import.
        sys.modules.pop("fleetlink", None)
        sys.path.insert(
            0, str(pathlib.Path(__file__).resolve().parents[2] / "fleetlink")
        )
        return pytest.importorskip("fleetlink.relay").RelayConfig
    return RelayConfig


def test_real_relay_yaml_loads_and_filters_platforms(relay_config_class):
    cfg = relay_config_class.load(RELAY_YAML)
    spot = cfg.resolve("euclid", "spot", base="willow")
    husky = cfg.resolve("gauss", "phoenix", base="willow")
    plan = "/euclid/omniplanner_node/compiled_plan_out"
    assert spot.topics[plan].type == "robot_executor_msgs/msg/ActionSequenceMsg"
    assert (
        husky.topics["/gauss/omniplanner_node/compiled_plan_out"].type
        == "nav_msgs/msg/Path"
    )
    assert "/euclid/spot_executor_node/pause" in spot.topics
    assert "/gauss/spot_executor_node/pause" not in husky.topics
    assert "/global/ros_system_monitor/table_in" in husky.topics  # no platform filter
    assert (
        "/euclid/omniplanner_node/llm_response" not in spot.topics
    )  # dead: not relayed
    hold = spot.services["/euclid/update_holding_state"]
    assert hold.base_name == "/euclid/update_holding_state" and hold.dir == "up"
    assert "/gauss/update_holding_state" not in husky.services
    # Stop & save: hydra then roman, both robots; no bare /<robot>/shutdown
    for robot, res in (("euclid", spot), ("gauss", husky)):
        for srv in ("hydra/shutdown", "roman/shutdown"):
            s = res.services[f"/{robot}/{srv}"]
            assert s.dir == "down" and s.type == "std_srvs/srv/Empty"
        assert f"/{robot}/shutdown" not in res.services
    # roman pickles its map before answering: the relay must wait for it
    assert spot.services["/euclid/roman/shutdown"].timeout_s >= 120
    take = spot.services["/euclid/spot_executor_node/take_lease"]
    assert take.dir == "down" and take.type == "std_srvs/srv/Trigger"
    assert "/gauss/spot_executor_node/take_lease" not in husky.services
    assert "/_tf/map/euclid/base_link" in spot.topics and "/_tf_static" in spot.topics
    assert len(cfg.hash) == 12


def test_real_relay_yaml_phoenix_rows(relay_config_class):
    cfg = relay_config_class.load(RELAY_YAML)
    spot = cfg.resolve("euclid", "spot", base="willow")
    for platform in ("phoenix", "husky"):  # husky: the topology alias
        phx = cfg.resolve("gauss", platform, base="willow")
        # Stop / Resume: toggle_manual_mode, a C0 down service like take_lease
        mode = phx.services["/gauss/toggle_manual_mode"]
        assert mode.dir == "down" and mode.type == "std_srvs/srv/SetBool"
        assert mode.cls == "C0" and mode.ttl_s <= 10
        # teleop into twist_mux: same strict 0.5 s ttl as the Spot cmd_vel row
        tw = phx.topics["/gauss/twist_marker_server/cmd_vel"]
        spot_tw = spot.topics["/euclid/spot_executor_node/cmd_vel"]
        assert tw.type == "geometry_msgs/msg/TwistStamped" and tw.dir == "down"
        assert (tw.mode, tw.cls, tw.ttl_s, tw.ttl_strict, tw.replay_on_resync) == (
            spot_tw.mode,
            spot_tw.cls,
            0.5,
            True,
            False,
        )
        mm = phx.topics["/gauss/manual_mode"]
        assert mm.dir == "up" and mm.type == "std_msgs/msg/Bool" and mm.mode == "latest"
        assert "/gauss/spot_executor_node/cmd_vel" not in phx.topics
        assert "/gauss/spot_executor_node/stand" not in phx.services
        assert "/_tf/map/gauss/base_link" in phx.topics
    # and none of it on a Spot
    assert "/euclid/toggle_manual_mode" not in spot.services
    assert "/euclid/twist_marker_server/cmd_vel" not in spot.topics
    assert "/euclid/manual_mode" not in spot.topics


def test_relay_yaml_teleop_entries(relay_config_class):
    cfg = relay_config_class.load(RELAY_YAML)
    spot = cfg.resolve("euclid", "spot", base="willow")
    husky = cfg.resolve("gauss", "phoenix", base="willow")
    vel = spot.topics["/euclid/spot_executor_node/cmd_vel"]
    assert vel.type == "geometry_msgs/msg/Twist" and vel.dir == "down"
    assert vel.mode == "latest" and vel.cls in ("C0", "C1")
    assert vel.ttl_s is not None and vel.ttl_s <= 0.5 and vel.ttl_strict
    assert vel.rate_hz and 5 <= vel.rate_hz <= 20
    assert not vel.replay_on_resync  # a restarted robot never replays motion
    assert "/gauss/spot_executor_node/cmd_vel" not in husky.topics
    for srv in ("stand", "sit"):
        s = spot.services[f"/euclid/spot_executor_node/{srv}"]
        assert s.dir == "down" and s.type == "std_srvs/srv/Trigger" and s.cls == "C0"
        assert s.ttl_s is not None and s.ttl_s <= 10 and s.timeout_s >= 10
        assert f"/gauss/spot_executor_node/{srv}" not in husky.services
    st = spot.topics["/euclid/spot_executor_node/teleop_state"]
    assert st.dir == "up" and st.type == "std_msgs/msg/String"


def test_plan_progress_fans_out_without_relaying_its_downlink_back_up(
    relay_config_class,
):
    cfg = relay_config_class.load(RELAY_YAML)
    for robot in ("hamilton", "euclid"):
        resolved = cfg.resolve(robot, "spot", "base_station")
        uplink = resolved.topics[f"/{robot}/spot_executor_node/plan_progress"]
        downlink = resolved.topics["/plan_progress"]
        assert uplink.dir == "up" and uplink.base_topic == downlink.topic
        assert downlink.dir == "down"
        assert uplink.mode == downlink.mode == "reliable"
        assert uplink.qos["depth"] == downlink.qos["depth"] == 100
        assert all(
            entry.topic != downlink.topic for entry in resolved.out_topics("robot")
        )


def test_mission_id_goes_down_latched_to_every_robot(relay_config_class):
    cfg = relay_config_class.load(RELAY_YAML)
    for robot, platform in (("euclid", "spot"), ("hamilton", "spot"), ("gauss", "phoenix")):
        res = cfg.resolve(robot, platform, base="willow")
        e = res.topics["/global/mission/current"]
        assert e.topic == "/global/mission/current"  # same name on every robot
        assert e.type == "std_msgs/msg/String" and e.dir == "down"
        assert e.mode == "latest_reliable" and e.replay_on_resync
        assert e.qos["durability"] == "transient_local" and e.ttl_s is None
