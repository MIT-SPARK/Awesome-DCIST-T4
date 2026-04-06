# ADT4 Network Diagnostics & Connectivity Toolkit

Tools for diagnosing and managing network connectivity across the ADT4 multi-robot system. Covers the Silvus mesh radio layer, Zenoh middleware, and ROS2 communication.

## Quick Start

```bash
# From any ADT4 machine with the repo checked out:
cd ~/dcist_ws/src/awesome_dcist_t4/network_diagnostics

# Pre-flight checklist (go/no-go before an experiment)
./scripts/preflight.sh --network silvus

# Run the full diagnostic suite
./scripts/net_diag.sh --network silvus

# Check Silvus radio battery/voltage/temp
./scripts/silvus_status.sh status

# Check Zenoh connectivity (auto-starts zenohd if not running)
source ~/dcist_ws/install/setup.zsh
./scripts/zenoh_check.sh

```

## Prerequisites

- Python 3 with PyYAML (included in all ROS2 installations)
- ROS2 workspace sourced (for Zenoh and ROS2 tools): `source ~/dcist_ws/install/setup.zsh`
- SSH keys set up for robot/base station access (for `ssh_check.sh`)
- Route to Silvus management network (for `silvus_status.sh`):
  ```bash
  # Find your Silvus ethernet interface
  ip -brief addr show | grep 192.168.100
  # Add route (replace enxXXX with your interface name)
  sudo ip route add 172.20.0.0/16 dev enxXXXXXXXXXXXX
  ```

## Configuration

All tools read from a single config file: **`config/topology.yaml`**

This file defines:
- **Networks**: MIT WiFi, Penn WiFi, Silvus subnets
- **Machines**: All robots and base stations with IPs per network
- **Silvus radios**: Management IPs (172.20.x.x), data IPs (192.168.100.x), node IDs
- **Zenoh**: Port and default connect network
- **SSH**: Per-role usernames

> **Note**: MIT WiFi IPs (10.29.x.x) are DHCP-assigned and change frequently.
> Silvus data IPs (192.168.100.x) and radio management IPs (172.20.x.x) are static.

## Tools Reference

### `preflight.sh` — Pre-Flight Checklist

Go/no-go checklist before experiments. Exits 0 (all pass) or 1 (failures). Checks:
1. Network interfaces detected
2. All peers pingable on selected network
3. Silvus radios reachable with battery above threshold
4. Zenoh router running and port listening
5. Remote Zenoh peers reachable
6. SSH access to all machines
7. ROS2 environment configured

```bash
# Full preflight on Silvus
./scripts/preflight.sh --network silvus

# Skip slow checks
./scripts/preflight.sh --network silvus --skip-ssh --skip-silvus

# Custom battery threshold
./scripts/preflight.sh --network silvus --battery-threshold 30
```

### `net_diag.sh` — Full Diagnostic Suite

Runs ping sweep, Zenoh check, and ROS2 status in sequence. Start here.

```bash
# Full diagnostics on Silvus network
./scripts/net_diag.sh --network silvus

# Skip ROS2 checks (no workspace sourced)
./scripts/net_diag.sh --network silvus --skip-ros

# Skip Zenoh checks
./scripts/net_diag.sh --network silvus --skip-zenoh

# All networks
./scripts/net_diag.sh
```

### `ping_sweep.sh` — Network Layer Diagnostics

Test basic IP connectivity, bandwidth, MTU, and routing.

```bash
# Ping all machines on Silvus
./scripts/ping_sweep.sh sweep --network silvus

# Ping all machines on all networks
./scripts/ping_sweep.sh sweep

# Bandwidth test (requires iperf3 -s running on target)
./scripts/ping_sweep.sh bandwidth 192.168.100.3

# MTU path discovery (important for Silvus — check for fragmentation)
./scripts/ping_sweep.sh mtu 192.168.100.3

# Trace route to target
./scripts/ping_sweep.sh traceroute 192.168.100.3
```

### `zenoh_check.sh` — Zenoh Middleware Diagnostics

Checks Zenoh router status without needing ROS2 topics running. Auto-starts zenohd if not running.

```bash
# Full Zenoh check (uses default network from topology.yaml)
source ~/dcist_ws/install/setup.zsh
./scripts/zenoh_check.sh

# Check on specific network
./scripts/zenoh_check.sh --network mit_wifi
```

**What it checks:**
1. Is `rmw_zenohd` process running? (starts it if not)
2. Is port 7447 listening locally?
3. Can we reach Zenoh port on each remote peer?
4. Zenoh admin space (router info, sessions, subscribers) — requires admin-enabled config

### `zenoh_watchdog.sh` — Zenoh Session Monitor

Continuously monitors Zenoh peer connections and alerts on connect/disconnect events.

```bash
# Default: poll every 5 seconds
./scripts/zenoh_watchdog.sh

# Custom interval and admin port
./scripts/zenoh_watchdog.sh --interval 10 --admin-port 8000
```

Output:
```
[14:30:15] Current sessions:
  * abc123def456 (router)
[14:30:20] + CONNECTED  789abc012345 (client)
[14:31:05] - DISCONNECTED  789abc012345 (client)
[14:31:05] ! PORT UNREACHABLE  euclid (192.168.100.3:7447)
```

### `gen_zenoh_config.sh` — Zenoh Router Config Generator

Generates a Zenoh router config with connect endpoints for all peers on a given network. Enables admin space and REST API for diagnostics.

```bash
# Generate config for Silvus network
./scripts/gen_zenoh_config.sh --network silvus > /tmp/zenoh_config.json5

# Start zenohd with generated config
ZENOH_ROUTER_CONFIG_URI=/tmp/zenoh_config.json5 ros2 run rmw_zenoh_cpp rmw_zenohd

# Query admin space (after starting with generated config)
curl http://localhost:8000/@/router/local
```

### `silvus_status.sh` — Silvus Radio Status & Diagnostics

Queries Silvus StreamCaster radios via their JSON-RPC API (StreamScape 5) at `/cgi-bin/streamscape_api`.

**First-time setup:**
```bash
# 1. Add route to radio management network
sudo ip route add 172.20.0.0/16 dev <your_silvus_interface>

# 2. Discover radio management IPs (if not in topology.yaml)
./scripts/silvus_status.sh discover

# 3. Identify which radio is connected to this machine
./scripts/silvus_status.sh detect

# 4. Update topology.yaml with discovered mgmt_ip values
```

**Commands:**
```bash
# Query all radios — battery, voltage, temp, model, mesh, GPS
./scripts/silvus_status.sh status

# Identify which radio is plugged into this machine (interactive, cached)
./scripts/silvus_status.sh detect

# Show mesh routing topology with management IPs
./scripts/silvus_status.sh topology

# RSSI link quality matrix between all radios
./scripts/silvus_status.sh rssi

# Monitor battery levels with alerts (threshold%, interval seconds)
./scripts/silvus_status.sh battery-watch 20 30

# Show detailed radio configuration (frequency, bandwidth, TX power, MIMO, encryption)
./scripts/silvus_status.sh radio-config

# Compare settings across all radios — highlights differences
./scripts/silvus_status.sh config-diff

# Discover radio management IPs via ARP scan
./scripts/silvus_status.sh discover

# Listen for UDP telemetry (RSSI, voltage, temperature)
./scripts/silvus_status.sh listen [port]
```

**Auto-detection:** Radios are not statically bound to machines — they can be swapped freely. The `detect` command uses a cache keyed by the USB-ethernet adapter's MAC address. Run `detect` once per adapter; the mapping persists across reboots and is invalidated if the adapter changes.

**Example output:**
```
=== Silvus Radio Status ===

Detecting local radio... found node 310061 at 172.20.187.45

Radio status (JSON-RPC API):

  Querying radio_310061 (172.20.187.45)... OK (3ms) (LOCAL)
    Model:    SC4200H (firmware v5.0.1.8)
    Node ID:  310061
    Battery:  91%  Voltage: 11.95V  Temp: 38°C
    Uptime:   1:50
    Mesh:     2 node(s) in routing tree: [310226, 310061]
    GPS:      unlocked

  Querying radio_310226 (172.20.187.210)... OK (18ms)
    Model:    SC4200H (firmware v5.0.1.8)
    Node ID:  310226
    Battery:  90%  Voltage: 12.03V  Temp: 39°C
    Uptime:   1:50
    Mesh:     2 node(s) in routing tree: [310061, 310226]
    GPS:      unlocked
```

**Available JSON-RPC methods** (discovered from StreamScape 5 web GUI):

| Method | Returns | Example |
|--------|---------|---------|
| `battery_percent` | Battery % (float string) | `["90.625000"]` |
| `input_voltage_monitoring` | Millivolts (float string) | `["11948.535156"]` |
| `read_current_temperature` | Celsius (int string) | `["38"]` |
| `nodeid` | Radio node ID | `["310061"]` |
| `model` | Hardware model | `["SC4200H"]` |
| `build_tag` | Firmware version | `["streamscape_v5.0.1.8"]` |
| `uptime` | System uptime string | `[" 01:47:16 up 1:47, ..."]` |
| `routing_tree` | Array of node IDs in mesh | `[310226, 310061]` |
| `gps_mode` | GPS lock status | `["unlocked"]` |
| `gps_coordinates` | [lat, lon, alt] | `["0", "0", "0"]` |
| `local_address` | Management IP | `["172.20.187.45"]` |
| `radio_mode` | Current radio mode | `["0"]` |
| `frequency` | Center frequency MHz | |
| `bandwidth` | Channel bandwidth MHz | |
| `tx_power` | Transmit power | |
| `encryption_mode` | Encryption setting | |
| `mimo_mode` | MIMO configuration | |

### `switch_network.sh` — WiFi / Silvus Switching

Switch between WiFi and Silvus networks. Manages interfaces and verifies connectivity.

```bash
# Switch to Silvus (disables WiFi, brings up Silvus interface)
./scripts/switch_network.sh --to silvus

# Switch to WiFi (enables WiFi via nmcli)
./scripts/switch_network.sh --to wifi

# Show Zenoh endpoint hints after switching
./scripts/switch_network.sh --to silvus --show-zenoh

# Check current network status
./scripts/switch_network.sh --status
```

### `link_monitor.sh` — Live Dashboard

Continuous color-coded ping dashboard. Useful during field testing to monitor link stability.

```bash
# Default: refresh every 2 seconds
./scripts/link_monitor.sh

# Custom refresh interval
./scripts/link_monitor.sh 5
```

### `ssh_check.sh` — SSH Access Verification

Tests SSH connectivity to all machines using the per-role users from topology.yaml.

```bash
# Check SSH to all machines (first available network)
./scripts/ssh_check.sh

# Check SSH on specific network
./scripts/ssh_check.sh --network silvus

# Custom timeout
./scripts/ssh_check.sh --network silvus --timeout 5
```

### `collect_logs.sh` — Diagnostic Snapshot

Captures a full diagnostic snapshot to a timestamped log file for post-mortem analysis. ANSI color codes are stripped.

```bash
# Full snapshot
./scripts/collect_logs.sh

# Specify network and output directory
./scripts/collect_logs.sh --network silvus --output-dir /tmp/diag

# Skip Silvus radio queries (faster if radios are off)
./scripts/collect_logs.sh --skip-silvus
```

**Captures:**
- System info (hostname, kernel, IPs, routes, interface stats)
- Ping sweep results
- Silvus radio status, config, config diff, topology
- Zenoh connectivity check
- ROS2 node and topic lists
- SSH access check

Output: `~/adt4_diagnostics/diag_<hostname>_<YYYYMMDD_HHMMSS>.log`

### ROS2 Connectivity Test (`ros_connectivity_test` package)

ROS2 pub/sub heartbeat test for verifying end-to-end middleware communication.

```bash
# Build the package
cd ~/dcist_ws
colcon build --packages-select ros_connectivity_test
source install/setup.zsh

# On machine A: publish heartbeats
ros2 run ros_connectivity_test connectivity_pub

# On machine B: subscribe and report
ros2 run ros_connectivity_test connectivity_sub

# Monitor topic bandwidth
ros2 run ros_connectivity_test bw_monitor
```

**Publisher** sends JSON heartbeats on `/$ADT4_ROBOT_NAME/connectivity_test/heartbeat` at 1 Hz:
```json
{"src": "euclid", "seq": 42, "timestamp": 1710612345.678}
```

**Subscriber** auto-discovers all heartbeat topics and reports per-source:
- Message count, dropped packets, loss rate
- Latency percentiles (avg, p50, p95, p99, min, max) over rolling 100-message window
- Alerts when a source goes silent (default: 5s threshold)

Publishes summary on `/connectivity_test/status`.

**Bandwidth Monitor** auto-discovers String topics and reports bytes/sec and msgs/sec per topic. Publishes on `/connectivity_test/bandwidth`.

**Parameters** (via `config/connectivity_test.yaml`):
- `publish_rate_hz`: Heartbeat rate (default: 1.0)
- `silent_threshold_sec`: Seconds before marking source silent (default: 5.0)
- `report_rate_hz`: Status report rate (default: 0.5)

## Typical Workflows

### Pre-experiment connectivity check

```bash
# Quick go/no-go
./scripts/preflight.sh --network silvus

# Or step by step:
./scripts/silvus_status.sh status       # Radio health
./scripts/ping_sweep.sh sweep --network silvus  # Peer reachability
source ~/dcist_ws/install/setup.zsh
./scripts/zenoh_check.sh                # Zenoh middleware
./scripts/net_diag.sh --network silvus  # Everything at once
```

### Debugging a connectivity issue

```bash
# Can we ping the target?
./scripts/ping_sweep.sh sweep --network silvus

# Is it an MTU issue? (common with Silvus)
./scripts/ping_sweep.sh mtu 192.168.100.3

# Is Zenoh running on both ends?
./scripts/zenoh_check.sh

# Check radio health (battery, overheating?)
./scripts/silvus_status.sh status

# Are the radios configured the same?
./scripts/silvus_status.sh config-diff

# Watch link stability over time
./scripts/link_monitor.sh 2

# Watch for Zenoh peer disconnects
./scripts/zenoh_watchdog.sh
```

### Post-experiment analysis

```bash
# Capture full diagnostic snapshot
./scripts/collect_logs.sh --network silvus

# Logs saved to ~/adt4_diagnostics/diag_<host>_<timestamp>.log
```

### Setting up Zenoh for a new network

```bash
# Generate config with all peers on Silvus
./scripts/gen_zenoh_config.sh --network silvus > /tmp/zenoh_config.json5

# Start zenohd with the config
ZENOH_ROUTER_CONFIG_URI=/tmp/zenoh_config.json5 ros2 run rmw_zenoh_cpp rmw_zenohd

# Verify connections
curl -s http://localhost:8000/@/router/local/link/** | python3 -m json.tool
```

### Discovering new Silvus radios

```bash
# Add management route
sudo ip route add 172.20.0.0/16 dev enx306893ab9ba5

# Run discovery
./scripts/silvus_status.sh discover

# Edit topology.yaml with found IPs, then verify
./scripts/silvus_status.sh status
```

## Launch System Integration

The connectivity pub/sub nodes are included in the `status` and `multi_status` tmux windows, so they run automatically with any experiment that includes `main`.

A standalone `net_diag` launch component is also available:

```yaml
# In experiment_manifest.yaml:
launch_configs:
    my_experiment: [core, net_diag, ...]
```

## Directory Structure

```
network_diagnostics/
├── config/
│   └── topology.yaml              # Network topology (IPs, roles, networks, radios)
├── scripts/
│   ├── net_diag.sh                # Full diagnostic suite
│   ├── preflight.sh               # Pre-flight go/no-go checklist
│   ├── collect_logs.sh            # Diagnostic snapshot to file
│   ├── ping_sweep.sh              # Ping, bandwidth, MTU, traceroute
│   ├── zenoh_check.sh             # Zenoh router status + auto-start
│   ├── zenoh_watchdog.sh          # Zenoh session connect/disconnect monitor
│   ├── gen_zenoh_config.sh        # Generate Zenoh config from topology
│   ├── silvus_status.sh           # Silvus radio JSON-RPC API client
│   ├── switch_network.sh          # WiFi ↔ Silvus switching
│   ├── link_monitor.sh            # Live ping dashboard
│   └── ssh_check.sh               # SSH access verification
├── ros_connectivity_test/         # ROS2 heartbeat pub/sub package
│   ├── ros_connectivity_test/
│   │   ├── pub_node.py            # Heartbeat publisher
│   │   ├── sub_node.py            # Heartbeat subscriber + latency reporter
│   │   └── bw_monitor_node.py     # Topic bandwidth monitor
│   ├── config/
│   │   └── connectivity_test.yaml
│   ├── package.xml
│   ├── setup.py
│   └── setup.cfg
└── launch_components/
    └── network_diagnostics.yaml   # Tmux launch component
```
