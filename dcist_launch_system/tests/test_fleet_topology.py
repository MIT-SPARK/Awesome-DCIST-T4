"""Tests for topology editing / comment-preserving save, and zenohd state."""

from unittest.mock import MagicMock

from dcist_launch_system.fleet import zenohd
from dcist_launch_system.fleet.topology import (
    apply_machine_edit,
    remove_machine,
    save_topology,
)
from dcist_launch_system.fleet_helpers import load_topology

import pytest

TOPO_TEXT = """---
# Fleet topology (hand-maintained)
networks:
  mit_wifi:
    desc: campus wifi
  silvus:
    desc: radio mesh
machines:
  euclid:
    role: robot
    platform_id: smaug
    platform_type: spot
    desc: Smaug Jockey
  # base station laptop
  willow:
    role: base_station
ssh:
  base_station_user: rrg
  robot_user: swarm
fleet_defaults:
  spot_platforms:
    - smaug
    - topaz
"""

ADDR_TEXT = """---
# private overlay (documentation addresses)
networks:
  mit_wifi:
    subnet: 198.51.100.0/24  # campus
  silvus:
    subnet: 203.0.113.0/24
machines:
  euclid:
    addresses:
      mit_wifi: 198.51.100.1
      penn_wifi: 192.0.2.234
      silvus: 203.0.113.3
  willow:
    addresses:
      mit_wifi: 198.51.100.9
"""


def _write(tmp_path):
    p = tmp_path / "topology.yaml"
    p.write_text(TOPO_TEXT)
    a = tmp_path / "fleet_addresses.yaml"
    a.write_text(ADDR_TEXT)
    return p, a


def test_noop_save_is_byte_identical(tmp_path):
    p, a = _write(tmp_path)
    save_topology(p, load_topology(p, a), a)
    assert p.read_text() == TOPO_TEXT
    assert a.read_text() == ADDR_TEXT


def test_edit_keeps_comments_extra_keys_and_other_networks(tmp_path):
    p, a = _write(tmp_path)
    topo = load_topology(p, a)
    runtime = {}
    apply_machine_edit(
        topo,
        runtime,
        "mit_wifi",
        {
            "name": "euclid",
            "role": "robot",
            "platform": "smaug",
            "desc": "Smaug Jockey v2",
            "mit_wifi": "198.51.100.42",
            "silvus": "",
        },
        original_name="euclid",
    )
    entry = topo["machines"]["euclid"]
    assert entry["platform_type"] == "spot"  # extra key preserved
    assert entry["addresses"] == {
        "penn_wifi": "192.0.2.234",
        "mit_wifi": "198.51.100.42",
    }
    assert (
        runtime["euclid"]["ip"] == "198.51.100.42" and runtime["euclid"]["user"] == "swarm"
    )

    save_topology(p, topo, a)
    text = p.read_text()
    assert "# Fleet topology (hand-maintained)" in text and "# base station laptop" in text
    assert "Smaug Jockey v2" in text
    assert "addresses" not in text and "subnet" not in text  # public stays address-free
    addrs = a.read_text()
    assert "# campus" in addrs and "198.51.100.42" in addrs
    assert "203.0.113.3" not in addrs  # silvus cleared
    assert a.stat().st_mode & 0o777 == 0o600
    assert load_topology(p, a) == topo


def test_rename_and_remove(tmp_path):
    p, a = _write(tmp_path)
    topo = load_topology(p, a)
    runtime = {"willow": {"name": "willow"}}
    apply_machine_edit(
        topo,
        runtime,
        "silvus",
        {"name": "oak", "role": "base_station", "mit_wifi": "198.51.100.9"},
        original_name="willow",
    )
    assert "willow" not in topo["machines"] and "oak" in topo["machines"]
    assert "willow" not in runtime and "oak" not in runtime  # no silvus address

    remove_machine(topo, runtime, "euclid")
    save_topology(p, topo, a)
    assert set(load_topology(p, a)["machines"]) == {"oak"}
    assert "198.51.100" not in p.read_text()


def test_save_without_overlay_refuses_addresses(tmp_path):
    """No overlay: an address edit is refused and topology.yaml is left untouched."""
    p, _ = _write(tmp_path)
    topo = load_topology(p)  # conftest hides any real overlay
    apply_machine_edit(topo, {}, "mit_wifi", {"name": "euclid", "role": "robot", "mit_wifi": "198.51.100.42"}, original_name="euclid")  # fmt: skip
    with pytest.raises(ValueError, match="overlay"):
        save_topology(p, topo)
    assert p.read_text() == TOPO_TEXT
    # without addresses it saves (public part only)
    topo["machines"]["euclid"].pop("addresses")
    topo["machines"]["euclid"]["desc"] = "renamed"
    save_topology(p, topo)
    assert "renamed" in p.read_text() and "198.51.100" not in p.read_text()


def test_zenohd_stop_and_cleanup(tmp_path):
    state = zenohd.new_state()
    assert not zenohd.is_running(state)
    proc = MagicMock()
    proc.poll.return_value = None
    cfg = tmp_path / "z.json5"
    cfg.write_text("{}")
    state.update(proc=proc, config=str(cfg))
    assert zenohd.is_running(state)
    zenohd.cleanup(state)
    proc.terminate.assert_called_once()
    assert state == {"proc": None, "config": None} and not cfg.exists()


def test_real_topology_robot_platforms(topology_path, example_addresses_path):
    """hamilton is an explicit spot; gauss is the phoenix Husky on the radio.

    The public topology carries no addresses; they come from the overlay (here
    the documentation-address example)."""
    from dcist_launch_system.fleet_helpers import platform_of

    machines = load_topology(topology_path)["machines"]
    assert machines["hamilton"]["platform_type"] == "spot"
    assert platform_of(machines["gauss"]) == "phoenix"
    assert "addresses" not in machines["gauss"]
    merged = load_topology(topology_path, example_addresses_path)["machines"]
    assert set(merged["gauss"]["addresses"]) == {"mit_wifi", "silvus"}
