"""Unit tests for fleet_helpers.py — all SSH/subprocess calls are mocked."""

import subprocess
from unittest.mock import MagicMock, patch

import pytest

from dcist_launch_system.fleet_helpers import (
    detect_network,
    get_remote_status,
    hash_remote_experiment,
    list_remote_experiments,
    load_topology,
    ping_host,
    resolve_machines,
    run_parallel,
    ssh_cmd,
)


# ---------------------------------------------------------------------------
# load_topology
# ---------------------------------------------------------------------------


class TestLoadTopology:
    def test_loads_real_file(self, topology_path):
        topo = load_topology(topology_path)
        assert "machines" in topo
        assert "networks" in topo
        assert "ssh" in topo
        assert "euclid" in topo["machines"]

    def test_missing_file_raises(self, tmp_path):
        with pytest.raises(FileNotFoundError):
            load_topology(tmp_path / "nonexistent.yaml")

    def test_loads_custom_file(self, tmp_path, sample_topology):
        import yaml

        f = tmp_path / "topo.yaml"
        f.write_text(yaml.dump(sample_topology))
        result = load_topology(f)
        assert result["machines"]["alpha"]["platform_id"] == "smaug"


# ---------------------------------------------------------------------------
# detect_network
# ---------------------------------------------------------------------------


class TestDetectNetwork:
    @patch("dcist_launch_system.fleet_helpers.subprocess.run")
    def test_detects_mit_wifi(self, mock_run, sample_topology):
        mock_run.return_value = MagicMock(
            stdout="2: wlan0    inet 10.29.5.42/24 brd 10.29.5.255 scope global\n"
        )
        assert detect_network(sample_topology) == "mit_wifi"

    @patch("dcist_launch_system.fleet_helpers.subprocess.run")
    def test_detects_silvus(self, mock_run, sample_topology):
        mock_run.return_value = MagicMock(
            stdout="3: eth1    inet 192.168.100.50/24 brd 192.168.100.255 scope global\n"
        )
        assert detect_network(sample_topology) == "silvus"

    @patch("dcist_launch_system.fleet_helpers.subprocess.run")
    def test_returns_none_for_unknown(self, mock_run, sample_topology):
        mock_run.return_value = MagicMock(
            stdout="2: eth0    inet 172.16.0.5/24 brd 172.16.0.255 scope global\n"
        )
        assert detect_network(sample_topology) is None

    @patch("dcist_launch_system.fleet_helpers.subprocess.run")
    def test_handles_multiple_interfaces(self, mock_run, sample_topology):
        mock_run.return_value = MagicMock(
            stdout=(
                "1: lo    inet 127.0.0.1/8 scope host lo\n"
                "2: eth0    inet 172.16.0.5/24 brd 172.16.0.255 scope global\n"
                "3: wlan0    inet 192.168.100.7/24 brd 192.168.100.255 scope global\n"
            )
        )
        assert detect_network(sample_topology) == "silvus"


# ---------------------------------------------------------------------------
# resolve_machines
# ---------------------------------------------------------------------------


class TestResolveMachines:
    def test_all_machines_on_mit_wifi(self, sample_topology):
        result = resolve_machines(sample_topology, "mit_wifi")
        names = [m["name"] for m in result]
        assert "alpha" in names
        assert "beta" in names
        assert "gamma" in names
        assert len(result) == 3

    def test_filter_by_silvus_excludes_beta(self, sample_topology):
        result = resolve_machines(sample_topology, "silvus")
        names = [m["name"] for m in result]
        assert "alpha" in names
        assert "gamma" in names
        assert "beta" not in names  # beta has no silvus address

    def test_filter_by_role_robot(self, sample_topology):
        result = resolve_machines(sample_topology, "mit_wifi", roles=["robot"])
        names = [m["name"] for m in result]
        assert "alpha" in names
        assert "beta" in names
        assert "gamma" not in names

    def test_filter_by_role_base_station(self, sample_topology):
        result = resolve_machines(sample_topology, "mit_wifi", roles=["base_station"])
        assert len(result) == 1
        assert result[0]["name"] == "gamma"

    def test_filter_by_name(self, sample_topology):
        result = resolve_machines(sample_topology, "mit_wifi", names=["alpha"])
        assert len(result) == 1
        assert result[0]["name"] == "alpha"

    def test_correct_user_assignment(self, sample_topology):
        result = resolve_machines(sample_topology, "mit_wifi")
        by_name = {m["name"]: m for m in result}
        assert by_name["alpha"]["user"] == "swarm"  # robot
        assert by_name["gamma"]["user"] == "rrg"  # base_station

    def test_platform_id_included(self, sample_topology):
        result = resolve_machines(sample_topology, "mit_wifi", names=["alpha"])
        assert result[0]["platform_id"] == "smaug"

    def test_empty_for_nonexistent_network(self, sample_topology):
        result = resolve_machines(sample_topology, "penn_wifi")
        assert result == []

    def test_missing_platform_id_defaults_empty(self, sample_topology):
        result = resolve_machines(sample_topology, "mit_wifi", names=["gamma"])
        assert result[0]["platform_id"] == ""


# ---------------------------------------------------------------------------
# ping_host
# ---------------------------------------------------------------------------


class TestPingHost:
    @patch("dcist_launch_system.fleet_helpers.subprocess.run")
    def test_ping_success(self, mock_run):
        mock_run.return_value = MagicMock(returncode=0)
        assert ping_host("10.29.1.1") is True

    @patch("dcist_launch_system.fleet_helpers.subprocess.run")
    def test_ping_failure(self, mock_run):
        mock_run.return_value = MagicMock(returncode=1)
        assert ping_host("10.29.1.1") is False

    @patch("dcist_launch_system.fleet_helpers.subprocess.run")
    def test_ping_timeout(self, mock_run):
        mock_run.side_effect = subprocess.TimeoutExpired(cmd="ping", timeout=4)
        assert ping_host("10.29.1.1") is False

    @patch("dcist_launch_system.fleet_helpers.subprocess.run")
    def test_ping_os_error(self, mock_run):
        mock_run.side_effect = OSError("No route to host")
        assert ping_host("10.29.1.1") is False


# ---------------------------------------------------------------------------
# ssh_cmd
# ---------------------------------------------------------------------------


class TestSshCmd:
    @patch("dcist_launch_system.fleet_helpers.subprocess.run")
    def test_ssh_success(self, mock_run):
        mock_run.return_value = MagicMock(
            returncode=0, stdout="euclid", stderr=""
        )
        rc, out, err = ssh_cmd("swarm", "10.29.1.1", "hostname")
        assert rc == 0
        assert out == "euclid"

    @patch("dcist_launch_system.fleet_helpers.subprocess.run")
    def test_ssh_failure(self, mock_run):
        mock_run.return_value = MagicMock(
            returncode=255, stdout="", stderr="Connection refused"
        )
        rc, out, err = ssh_cmd("swarm", "10.29.1.1", "hostname")
        assert rc == 255
        assert "refused" in err.lower()

    @patch("dcist_launch_system.fleet_helpers.subprocess.run")
    def test_ssh_timeout(self, mock_run):
        mock_run.side_effect = subprocess.TimeoutExpired(cmd="ssh", timeout=15)
        rc, out, err = ssh_cmd("swarm", "10.29.1.1", "hostname")
        assert rc == -1
        assert "timeout" in err.lower()

    @patch("dcist_launch_system.fleet_helpers.subprocess.run")
    def test_ssh_os_error(self, mock_run):
        mock_run.side_effect = OSError("ssh not found")
        rc, out, err = ssh_cmd("swarm", "10.29.1.1", "hostname")
        assert rc == -1

    @patch("dcist_launch_system.fleet_helpers.subprocess.run")
    def test_ssh_batch_mode_flag(self, mock_run):
        mock_run.return_value = MagicMock(returncode=0, stdout="", stderr="")
        ssh_cmd("swarm", "10.29.1.1", "hostname")
        args = mock_run.call_args[0][0]
        assert "BatchMode=yes" in " ".join(args)


# ---------------------------------------------------------------------------
# list_remote_experiments
# ---------------------------------------------------------------------------


class TestListRemoteExperiments:
    @patch("dcist_launch_system.fleet_helpers.ssh_cmd")
    def test_parses_experiment_list(self, mock_ssh):
        mock_ssh.return_value = (
            0,
            "03192026_atak_test|hydra,roman,spot_executor|1.2G\n"
            "03202026_mapping|hydra,roman|800M",
            "",
        )
        result = list_remote_experiments("swarm", "10.29.1.1")
        assert len(result) == 2
        assert result[0]["name"] == "03192026_atak_test"
        assert result[0]["subdirs"] == ["hydra", "roman", "spot_executor"]
        assert result[0]["size"] == "1.2G"
        assert result[1]["name"] == "03202026_mapping"

    @patch("dcist_launch_system.fleet_helpers.ssh_cmd")
    def test_empty_output(self, mock_ssh):
        mock_ssh.return_value = (0, "", "")
        result = list_remote_experiments("swarm", "10.29.1.1")
        assert result == []

    @patch("dcist_launch_system.fleet_helpers.ssh_cmd")
    def test_ssh_failure(self, mock_ssh):
        mock_ssh.return_value = (1, "", "Connection refused")
        result = list_remote_experiments("swarm", "10.29.1.1")
        assert result == []

    @patch("dcist_launch_system.fleet_helpers.ssh_cmd")
    def test_experiment_with_no_subdirs(self, mock_ssh):
        mock_ssh.return_value = (0, "empty_exp||50M", "")
        result = list_remote_experiments("swarm", "10.29.1.1")
        assert len(result) == 1
        assert result[0]["subdirs"] == []
        assert result[0]["size"] == "50M"


# ---------------------------------------------------------------------------
# hash_remote_experiment
# ---------------------------------------------------------------------------


class TestHashRemoteExperiment:
    @patch("dcist_launch_system.fleet_helpers.ssh_cmd")
    def test_parses_md5sums(self, mock_ssh):
        mock_ssh.return_value = (
            0,
            "abc123def456  ./hydra/scene_graph.json\n"
            "789012345678  ./roman/map.pkl",
            "",
        )
        result = hash_remote_experiment("swarm", "10.29.1.1", "~/adt4_output", "test_exp")
        assert result["./hydra/scene_graph.json"] == "abc123def456"
        assert result["./roman/map.pkl"] == "789012345678"

    @patch("dcist_launch_system.fleet_helpers.ssh_cmd")
    def test_empty_on_failure(self, mock_ssh):
        mock_ssh.return_value = (1, "", "No such directory")
        result = hash_remote_experiment("swarm", "10.29.1.1", "~/adt4_output", "test_exp")
        assert result == {}

    @patch("dcist_launch_system.fleet_helpers.ssh_cmd")
    def test_empty_experiment(self, mock_ssh):
        mock_ssh.return_value = (0, "", "")
        result = hash_remote_experiment("swarm", "10.29.1.1", "~/adt4_output", "test_exp")
        assert result == {}


# ---------------------------------------------------------------------------
# get_remote_status
# ---------------------------------------------------------------------------


class TestGetRemoteStatus:
    @patch("dcist_launch_system.fleet_helpers.ssh_cmd")
    def test_parses_status(self, mock_ssh):
        mock_ssh.return_value = (
            0,
            "---TMUX---\n"
            "adt4_system: 5 windows (created Mon Mar 19 10:00:00 2026)\n"
            "---ROS2---\n"
            "42\n"
            "---LOAD---\n"
            "0.50, 0.75, 0.80\n"
            "---DISK---\n"
            "50G / 200G (75% used)",
            "",
        )
        result = get_remote_status("swarm", "10.29.1.1")
        assert "adt4_system" in result["tmux_sessions"]
        assert result["ros2_procs"] == "42"
        assert "0.50" in result["load"]
        assert "50G" in result["disk"]

    @patch("dcist_launch_system.fleet_helpers.ssh_cmd")
    def test_no_tmux_sessions(self, mock_ssh):
        mock_ssh.return_value = (
            0,
            "---TMUX---\nnone\n---ROS2---\n0\n---LOAD---\n0.10, 0.05, 0.01\n---DISK---\n100G / 200G (50% used)",
            "",
        )
        result = get_remote_status("swarm", "10.29.1.1")
        assert result["tmux_sessions"] == "none"
        assert result["ros2_procs"] == "0"

    @patch("dcist_launch_system.fleet_helpers.ssh_cmd")
    def test_ssh_failure(self, mock_ssh):
        mock_ssh.return_value = (-1, "", "SSH timeout")
        result = get_remote_status("swarm", "10.29.1.1")
        assert result["tmux_sessions"] == "N/A"
        assert result["ros2_procs"] == "N/A"


# ---------------------------------------------------------------------------
# run_parallel
# ---------------------------------------------------------------------------


class TestRunParallel:
    def test_basic_parallel(self):
        results = run_parallel(lambda x: x * 2, [1, 2, 3])
        result_map = {item: res for item, res in results}
        assert result_map[1] == 2
        assert result_map[2] == 4
        assert result_map[3] == 6

    def test_exception_handling(self):
        def sometimes_fail(x):
            if x == 2:
                raise ValueError("fail")
            return x

        results = run_parallel(sometimes_fail, [1, 2, 3])
        result_map = {item: res for item, res in results}
        assert result_map[1] == 1
        assert result_map[3] == 3
        assert isinstance(result_map[2], ValueError)

    def test_empty_input(self):
        results = run_parallel(lambda x: x, [])
        assert results == []


# ---------------------------------------------------------------------------
# Integration tests (require network, skip by default)
# ---------------------------------------------------------------------------


@pytest.mark.integration
class TestIntegrationPing:
    def test_localhost_reachable(self):
        assert ping_host("127.0.0.1", timeout=1) is True

    def test_unreachable_ip(self):
        assert ping_host("192.0.2.1", timeout=1) is False  # TEST-NET, not routable
