"""Tests for the public topology + private fleet-address overlay merge
(network_diagnostics/scripts/topology_merge.py, via fleet_helpers)."""

import copy
import logging
import os
import subprocess
import sys
from unittest.mock import MagicMock, patch

import pytest
import yaml

from dcist_launch_system import fleet_helpers
from dcist_launch_system.fleet_helpers import (
    detect_network,
    find_addresses_path,
    load_topology,
    resolve_machines,
    save_topology,
    topology_merge,
)


@pytest.fixture
def overlay_file(tmp_path, sample_topology):
    """Split sample_topology into public/private files; returns (public, private, merged)."""
    public, private = topology_merge.split_private(sample_topology)
    pub = tmp_path / "topology.yaml"
    priv = tmp_path / "fleet_addresses.yaml"
    pub.write_text(yaml.safe_dump(public, sort_keys=False))
    priv.write_text(yaml.safe_dump(private, sort_keys=False))
    return pub, priv, sample_topology


# ---------------------------------------------------------------------------
# deep_merge
# ---------------------------------------------------------------------------


class TestDeepMerge:
    def test_nested_overlay_wins_and_keeps_base_keys(self):
        base = {"a": {"x": 1, "y": {"p": 1}}, "b": 2}
        overlay = {"a": {"y": {"p": 9, "q": 3}, "z": 4}, "c": 5}
        assert topology_merge.deep_merge(base, overlay) == {
            "a": {"x": 1, "y": {"p": 9, "q": 3}, "z": 4},
            "b": 2,
            "c": 5,
        }

    def test_base_key_order_first(self):
        base = {"networks": {"mit_wifi": {}, "penn_wifi": {}, "silvus": {}}}
        overlay = {
            "networks": {
                "silvus": {"subnet": "203.0.113.0/24"},
                "mit_wifi": {"subnet": "198.51.100.0/24"},
            }
        }
        merged = topology_merge.deep_merge(base, overlay)
        assert list(merged["networks"]) == ["mit_wifi", "penn_wifi", "silvus"]

    def test_non_dict_overlay_replaces(self):
        merged = topology_merge.deep_merge({"a": {"x": 1}}, {"a": [1, 2]})
        assert merged == {"a": [1, 2]}

    def test_inputs_not_modified(self):
        base = {"a": {"x": 1}}
        overlay = {"a": {"y": 2}}
        topology_merge.deep_merge(base, overlay)
        assert base == {"a": {"x": 1}} and overlay == {"a": {"y": 2}}


# ---------------------------------------------------------------------------
# overlay lookup precedence
# ---------------------------------------------------------------------------


class TestFindAddressesPath:
    def test_none_when_nothing_configured(self):
        assert find_addresses_path() is None

    def test_default_path_used_if_it_exists(self, tmp_path, monkeypatch):
        default = tmp_path / "default.yaml"
        default.write_text("{}")
        monkeypatch.setattr(topology_merge, "DEFAULT_ADDRESSES_PATH", default)
        assert find_addresses_path() == default

    def test_env_beats_default(self, tmp_path, monkeypatch):
        default = tmp_path / "default.yaml"
        env = tmp_path / "env.yaml"
        default.write_text("{}")
        env.write_text("{}")
        monkeypatch.setattr(topology_merge, "DEFAULT_ADDRESSES_PATH", default)
        monkeypatch.setenv("ADT4_FLEET_ADDRESSES", str(env))
        assert find_addresses_path() == env

    def test_argument_beats_env(self, tmp_path, monkeypatch):
        env = tmp_path / "env.yaml"
        arg = tmp_path / "arg.yaml"
        env.write_text("{}")
        arg.write_text("{}")
        monkeypatch.setenv("ADT4_FLEET_ADDRESSES", str(env))
        assert find_addresses_path(arg) == arg

    def test_explicit_missing_raises(self, tmp_path, monkeypatch):
        with pytest.raises(FileNotFoundError):
            find_addresses_path(tmp_path / "nope.yaml")
        monkeypatch.setenv("ADT4_FLEET_ADDRESSES", str(tmp_path / "nope.yaml"))
        with pytest.raises(FileNotFoundError):
            find_addresses_path()

    def test_load_topology_uses_env(self, overlay_file, monkeypatch):
        pub, priv, merged = overlay_file
        monkeypatch.setenv("ADT4_FLEET_ADDRESSES", str(priv))
        assert load_topology(pub) == merged


# ---------------------------------------------------------------------------
# load_topology with and without an overlay
# ---------------------------------------------------------------------------


class TestLoadMerged:
    def test_merges_overlay(self, overlay_file):
        pub, priv, merged = overlay_file
        topo = load_topology(pub, priv)
        assert topo == merged
        assert resolve_machines(topo, "silvus")[0]["ip"] == "203.0.113.10"

    def test_missing_overlay_warns_once_and_loads(self, overlay_file, caplog):
        pub, _, merged = overlay_file
        with caplog.at_level(logging.WARNING, logger=fleet_helpers.__name__):
            topo = load_topology(pub)
            load_topology(pub)
        warnings = [r for r in caplog.records if "fleet address overlay" in r.message]
        assert len(warnings) == 1
        assert set(topo["machines"]) == set(merged["machines"])
        assert "addresses" not in topo["machines"]["alpha"]
        assert resolve_machines(topo, "mit_wifi") == []

    @patch("dcist_launch_system.fleet_helpers.subprocess.run")
    def test_detect_network_without_subnets(self, mock_run, overlay_file):
        mock_run.return_value = MagicMock(
            stdout="2: wlan0    inet 198.51.100.42/24 brd 198.51.100.255 scope global\n"
        )
        pub, _, _ = overlay_file
        assert detect_network(load_topology(pub)) is None

    def test_real_topology_has_no_private_keys(self, topology_path):
        """The public topology.yaml must not carry any address."""
        public = yaml.safe_load(open(topology_path))
        _, private = topology_merge.split_private(public)
        assert private == {}

    def test_real_topology_with_example_overlay(
        self, topology_path, example_addresses_path
    ):
        topo = load_topology(topology_path, example_addresses_path)
        public = yaml.safe_load(open(topology_path))
        example = yaml.safe_load(open(example_addresses_path))
        # Every overlay machine exists in the public topology, and vice versa.
        assert set(example["machines"]) == set(public["machines"])
        assert list(topo["networks"]) == list(public["networks"])
        for net in topo["networks"].values():
            assert net["subnet"]


# ---------------------------------------------------------------------------
# split_private / save_topology
# ---------------------------------------------------------------------------


class TestSplitPrivate:
    def test_round_trip_sample(self, sample_topology):
        public, private = topology_merge.split_private(sample_topology)
        assert topology_merge.deep_merge(public, private) == sample_topology
        assert "addresses" not in public["machines"]["alpha"]
        assert "subnet" not in public["networks"]["silvus"]
        assert list(public["networks"]) == ["mit_wifi", "silvus"]

    def test_round_trip_real_with_example(self, topology_path, example_addresses_path):
        merged = load_topology(topology_path, example_addresses_path)
        public, private = topology_merge.split_private(merged)
        assert topology_merge.deep_merge(public, private) == merged
        assert private["silvus_radios"]["radios"]["radio_100001"]["node_id"] == 100001
        assert "radios" not in public["silvus_radios"]
        assert "mgmt_subnet" not in public["silvus_radios"]
        assert public["silvus_radios"]["api_endpoint"]

    def test_radio_with_public_keys_kept(self):
        topo = {
            "silvus_radios": {
                "radios": {"r1": {"mgmt_ip": "203.0.113.9", "node_id": 1, "label": "x"}}
            }
        }
        public, private = topology_merge.split_private(topo)
        assert public == {"silvus_radios": {"radios": {"r1": {"label": "x"}}}}
        assert topology_merge.deep_merge(public, private) == topo

    def test_input_not_modified(self, sample_topology):
        before = copy.deepcopy(sample_topology)
        topology_merge.split_private(sample_topology)
        assert sample_topology == before

    def test_save_topology_without_overlay(self, tmp_path, sample_topology):
        pub = tmp_path / "topology.yaml"
        _, private = save_topology(sample_topology, pub, None)
        assert private
        text = pub.read_text()
        assert "198.51.100" not in text and "203.0.113" not in text
        assert list(tmp_path.iterdir()) == [pub]


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------


def _run_cli(script, *args, env_extra=None, home=None):
    env = {k: v for k, v in os.environ.items() if k != "ADT4_FLEET_ADDRESSES"}
    env["HOME"] = str(home)
    env.update(env_extra or {})
    return subprocess.run(
        [sys.executable, str(script), *map(str, args)],
        capture_output=True,
        text=True,
        env=env,
    )


class TestCli:
    def test_out_with_overlay(self, topology_merge_script, overlay_file, tmp_path):
        pub, priv, merged = overlay_file
        out = tmp_path / "run" / "merged.yaml"
        r = _run_cli(
            topology_merge_script,
            "--topology",
            pub,
            "--out",
            out,
            env_extra={"ADT4_FLEET_ADDRESSES": str(priv)},
            home=tmp_path,
        )
        assert r.returncode == 0, r.stderr
        assert r.stdout.strip() == str(out)
        assert r.stderr == ""
        assert yaml.safe_load(out.read_text()) == merged
        assert os.stat(out).st_mode & 0o777 == 0o600

    def test_out_without_overlay_warns(
        self, topology_merge_script, overlay_file, tmp_path
    ):
        pub, _, _ = overlay_file
        out = tmp_path / "merged.yaml"
        r = _run_cli(
            topology_merge_script, "--topology", pub, "--out", out, home=tmp_path
        )
        assert r.returncode == 0
        assert "No fleet address overlay" in r.stderr
        assert yaml.safe_load(out.read_text()) == yaml.safe_load(pub.read_text())

    def test_out_replaces_existing_file(
        self, topology_merge_script, overlay_file, tmp_path
    ):
        pub, priv, merged = overlay_file
        out = tmp_path / "merged.yaml"
        out.write_text("old")
        os.chmod(out, 0o644)
        r = _run_cli(
            topology_merge_script,
            "--topology",
            pub,
            "--addresses",
            priv,
            "--out",
            out,
            home=tmp_path,
        )
        assert r.returncode == 0, r.stderr
        assert yaml.safe_load(out.read_text()) == merged
        assert os.stat(out).st_mode & 0o777 == 0o600

    def test_check(self, topology_merge_script, overlay_file, tmp_path):
        pub, priv, _ = overlay_file
        r = _run_cli(topology_merge_script, "--topology", pub, "--check", home=tmp_path)
        assert r.returncode == 1
        assert "ADT4_FLEET_ADDRESSES" in r.stderr
        r = _run_cli(
            topology_merge_script,
            "--topology",
            pub,
            "--check",
            env_extra={"ADT4_FLEET_ADDRESSES": str(priv)},
            home=tmp_path,
        )
        assert r.returncode == 0, r.stderr
        assert r.stdout.strip() == str(priv)

    def test_check_finds_default_path(
        self, topology_merge_script, overlay_file, tmp_path
    ):
        pub, priv, _ = overlay_file
        default = (
            tmp_path / ".config" / "adt4" / "fleet_addresses" / "fleet_addresses.yaml"
        )
        default.parent.mkdir(parents=True)
        default.write_text(priv.read_text())
        r = _run_cli(topology_merge_script, "--topology", pub, "--check", home=tmp_path)
        assert r.returncode == 0, r.stderr

    def test_missing_explicit_overlay_fails(
        self, topology_merge_script, overlay_file, tmp_path
    ):
        pub, _, _ = overlay_file
        r = _run_cli(
            topology_merge_script,
            "--topology",
            pub,
            "--out",
            tmp_path / "m.yaml",
            env_extra={"ADT4_FLEET_ADDRESSES": str(tmp_path / "nope.yaml")},
            home=tmp_path,
        )
        assert r.returncode == 1
        assert "not found" in r.stderr
        assert not (tmp_path / "m.yaml").exists()
