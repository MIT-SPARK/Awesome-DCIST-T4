# fleet-adt4 User Guide

`fleet-adt4` is a terminal UI (TUI) for managing multi-robot field deployments. It handles launching experiment sessions on robots, monitoring fleet health, transferring map data, and configuring network topology.

## Quick Start

```bash
# Auto-detect network, load default topology
fleet-adt4

# Specify network explicitly
fleet-adt4 --network silvus

# Use a different topology file and/or fleet address overlay
fleet-adt4 --topology /path/to/topology.yaml --addresses /path/to/fleet_addresses.yaml
```

Machine addresses are not in `topology.yaml`; they come from the private fleet
address overlay (see [Fleet addresses (private)](#fleet-addresses-private)).

## Navigation Overview

```
Main Screen
├── l → Launch Screen         (configure and start experiment sessions)
├── b → Maps Screen           (browse and transfer experiments)
├── c → Config Screen         (edit topology: add/remove/modify machines)
├── o → Monitor Screen        (runtime status, ROS2 nodes, bandwidth)
├── z → Zenoh Screen          (deploy Zenoh router configs)
├── s   Refresh connectivity
└── q   Quit
```

Press **Escape** from any screen to go back to the previous screen.

---

## Main Screen

The main screen shows a live status table for all machines defined in `topology.yaml`.

| Status | Meaning |
|--------|---------|
| **Online** (green) | Machine is reachable via ping and SSH |
| **No SSH** (yellow) | Pings but SSH fails |
| **Offline** (red) | Not reachable |

| Key | Action |
|-----|--------|
| `s` | Re-check all machines |
| `l` | Open Launch Screen |
| `b` | Open Maps Screen |
| `c` | Open Config Screen |
| `o` | Open Monitor Screen |
| `z` | Open Zenoh Config Screen |
| `q` | Quit |

---

## Launch Screen (`l`)

Configure and start experiment sessions across the fleet.

### Workflow

1. Press `m` (mapping) or `r` (relocalize) to auto-populate defaults from `topology.yaml`
2. Verify the robot and session selections
3. Select a base station (optional) and its session
4. Enter an experiment name (output saved to `~/adt4_output/<name>/runs/<run_id>/`, with `~/adt4_output/<name>/latest` pointing at the newest run)
5. For relocalize mode: select a prior map from the fleet or enter a custom path
6. Press `d` to preview the SSH commands (dry run), or `g` to launch

| Key | Action |
|-----|--------|
| `m` | Apply mapping preset (auto-selects robots, sessions) |
| `r` | Apply relocalize preset (auto-selects robots, sessions) |
| `d` | Dry run — show commands without executing |
| `g` | Launch — execute the experiment |
| `z` | Deploy Zenoh config to base station |
| `p` | Refresh prior map list from fleet |
| `F2` | Toggle log panel size |

### Launch Phases

When you press `g`, the launch proceeds in phases:

1. **Phase 0** — Deploy Zenoh `connect.endpoints` to the base station
2. **Phase 1** — Transfer prior maps to robots that need them (relocalize only)
3. **Phase 1.5** — Kill any existing conflicting tmux sessions (prompts for confirmation)
4. **Phase 2** — Check for existing output directories (an existing experiment keeps its earlier runs; each launch adds a new `runs/<run_id>`)
5. **Phase 2/3** — SSH into each robot and start `run-adt4`; then launch base station session

### Auto-Population (Presets)

The `fleet_defaults` section in `topology.yaml` controls what gets pre-selected when you press `m` or `r`:

```yaml
fleet_defaults:
  spot_platforms: [smaug, topaz]        # these use "spot" sessions; others use "phoenix"
  sessions:
    mapping:
      spot: "spot-default"
      phoenix: "phoenix-default"
    relocalize:
      spot: "spot_relocalize-relocalize"
      phoenix: "phoenix_relocalize-relocalize"
  base_station_session: "base_station_heracles-default"
  output_root: "~/adt4_output"
```

---

## Maps Screen (`b`)

Browse all experiments stored on fleet machines and transfer them between machines.

| Key | Action |
|-----|--------|
| `r` | Refresh experiment list from fleet |
| `t` | Transfer selected experiment |
| `v` | Verify selected experiment (checksum comparison) |
| Escape | Back to main |

### Transfer Screen

1. Source machine and experiment are pre-selected from the Maps Screen
2. Select destination machines from the list
3. Press `t` to start — progress is shown in the log
4. `recorded_data/` (rosbags) is excluded by default

### Verify Screen

Computes checksums of all experiment files across machines. Highlights files that differ or are missing on some machines.

---

## Config Screen (`c`)

Edit the network topology: add, modify, or remove machines. `F5` saves the changes: addresses go to the fleet address overlay that was loaded, everything else to `topology.yaml`. If no overlay was loaded, the address changes are not saved (the log says so); they are never written to `topology.yaml`.

| Key | Action |
|-----|--------|
| `e` or `Enter` | Edit selected machine |
| `F2` | Add new machine |
| `F3` | Recheck connectivity of all machines |
| `F4` | Remove selected machine |
| `F5` | Save changes (`topology.yaml` + fleet address overlay) |
| Escape | Back (discards unsaved changes) |

### Machine Fields

| Field | Description |
|-------|-------------|
| Name | Machine identifier (e.g., `euclid`) — fixed after creation |
| Role | `robot` or `base_station` |
| Platform | Platform ID (e.g., `smaug`, `topaz`, `samoyed`) — used for session auto-selection |
| MIT WiFi IP | Address on `mit_wifi` network (10.29.x.x) |
| Silvus IP | Address on `silvus` network (192.168.100.x) |
| Description | Free-text label |

---

## Monitor Screen (`o`)

Runtime status and diagnostics for the whole fleet.

| Key | Action |
|-----|--------|
| `r` | Refresh system status (tmux, ROS2, load, disk, GPU, battery, radio) |
| `n` | Refresh ROS2 node readiness |
| `Enter` | Drill down into selected robot's node details |
| `v` | Generate per-robot RViz config |
| `z` | Check Zenoh port reachability + Silvus link quality |
| `F3` | Check Silvus management routes (shows commands to add if missing) |
| `F2` | Start/restart fleet-managed `zenohd` |
| `w` | Run bandwidth test (iperf3) |
| `k` | Stop node poller and save logs |
| Escape | Back to main |

### System Status Table

Each row shows one machine with: tmux session count, ROS2 process count, load average (1/5/15 min), memory usage, GPU status, disk usage, battery level, and Silvus radio signal quality.

### Node Readiness Table

Summarizes per-robot ROS2 node health. Color codes:

| Color | Meaning |
|-------|---------|
| Green | All expected nodes healthy |
| Yellow | Some nodes degraded or slow |
| Red | Critical nodes missing or failed |

Press `Enter` on a robot row to see the full per-node breakdown.

### Silvus Routes (`F3`)

Checks whether the management route (`172.20.0.0/16`) exists on each machine. If missing, shows the exact commands to run:

```
# Temporary (cleared on reboot):
sudo ip route add 172.20.0.0/16 dev <iface>

# Permanent (same USB dongle):
conn=$(nmcli -f NAME,DEVICE con show | grep <iface> | awk '{print $1}')
nmcli con modify "$conn" +ipv4.routes "172.20.0.0/16"
nmcli con up "$conn"
```

### Bandwidth Test (`w`)

1. Select which online robots to test
2. Press `F5` to start — live per-second throughput is shown in the test window
3. Results appear in the monitor log after all tests complete

| Color | Meaning |
|-------|---------|
| Green | > 10 Mbps |
| Yellow | 1–10 Mbps |
| Red | < 1 Mbps |

If a direct connection is blocked (e.g., MIT WiFi client isolation), the test falls back to an SSH reverse tunnel. Results are marked as tunneled in that case.

---

## Zenoh Screen (`z`)

Manage Zenoh router configurations across the fleet.

| Key | Action |
|-----|--------|
| `d` | Deploy connect endpoints to all online machines |
| `p` | Preview endpoints for the selected machine |
| Escape | Back |

The fleet uses a star topology: the base station connects to all robots; robots listen only and need no config changes. Endpoint lists are generated from the active network's machine IPs and deployed by patching the existing JSON5 config files on each machine.

In fleetlink mode (`fleet_defaults.fleet_link: fleetlink`) no ROS router link crosses the radio: every machine gets an empty endpoint list (deploying it clears the base's robot links left from router mode; restart the base's `rmw_zenohd` afterwards), and the table shows each machine's fleetlink link instead. Robots reach the base only through fleetlink: each robot's agent dials the fleetlink router on `fleet_defaults.fleetlink_base`. See the README's Fleetlink section.

---

## topology.yaml Reference

Default location: `network_diagnostics/config/topology.yaml` (public, no addresses).
`fleet-adt4` merges the private fleet address overlay over it; the merged result
looks like this (addresses from `network_diagnostics/config/fleet_addresses.example.yaml`,
documentation ranges only):

```yaml
networks:              # order = detection priority
  mit_wifi:   { subnet: "198.51.100.0/24" }   # overlay
  silvus:     { subnet: "203.0.113.0/24" }    # overlay

machines:
  euclid:
    role: robot
    platform_id: smaug
    platform_type: spot
    desc: "Smaug Jockey"
    addresses:                 # overlay
      mit_wifi: "198.51.100.3"
      silvus: "203.0.113.3"
  sequia:
    role: base_station
    desc: "Base station"
    addresses:                 # overlay
      silvus: "203.0.113.105"

ssh:
  robot_user: "swarm"
  base_station_user: "rrg"

zenoh:
  port: 7447
  connect_network: silvus
  fleetlink_port: 7450          # the base's fleetlink router (fleetlink mode)
  fleetlink_networks: [silvus]  # networks robot agents dial it on, in failover order

fleet_defaults:
  spot_platforms: [smaug, topaz]
  sessions:
    mapping:
      spot: "spot-default"
      phoenix: "phoenix-default"
    relocalize:
      spot: "spot_relocalize-relocalize"
      phoenix: "phoenix_relocalize-relocalize"
  base_station_session: "base_station_heracles-default"
  output_root: "~/adt4_output"
  fleet_link: fleetlink   # fleetlink | router (see the README's Fleetlink section)
  fleetlink_base: willow  # base station that runs the fleetlink router

silvus_radios:
  api_endpoint: /cgi-bin/streamscape_api
  mgmt_subnet: "172.20.0.0/16"   # overlay (Silvus factory default)
  radios:                        # overlay
    radio_100001: { mgmt_ip: "172.20.134.161", node_id: 100001 }
```

`run-adt4` derives each session's fleet link values from this file merged with the address overlay below (`fleet_helpers.load_topology`, then `fleet_link_env`).

### Fleet addresses (private)

The overlay is `fleet_addresses.yaml` from the private repo
`robustrobotics/adt4_fleet_addresses`. It has the same layout as `topology.yaml`
and holds only `networks.<net>.subnet`, `machines.<name>.addresses`,
`silvus_radios.mgmt_subnet` and `silvus_radios.radios.<radio>.{mgmt_ip,node_id}`.
`fleet-adt4` uses, in order: `--addresses PATH`, `$ADT4_FLEET_ADDRESSES`,
`~/.config/adt4/fleet_addresses/fleet_addresses.yaml`.

```bash
git clone git@github.com:robustrobotics/adt4_fleet_addresses.git ~/.config/adt4/fleet_addresses
echo 'export ADT4_FLEET_ADDRESSES=~/.config/adt4/fleet_addresses/fleet_addresses.yaml' >> ~/.zshrc
```

Without an overlay `fleet-adt4` warns and no machine has an address. Edit
addresses in the overlay, or in the Config screen (`F5` saves them to the overlay);
a new machine goes in `topology.yaml` (role, platform) plus its `addresses` in the
overlay, under the same name. The `network_diagnostics` scripts use the same overlay.

### Network auto-detection

On startup, `fleet-adt4` checks the local machine's IP addresses against the subnets defined in `topology.yaml`. Networks are checked in the order they appear in the file — the first matching network becomes the `active_network`. With the default topology, the priority is: **mit_wifi → penn_wifi → silvus**. If you're connected to both MIT WiFi and Silvus at the same time, MIT WiFi will be selected.

Override with `--network <name>` to force a specific network:
```bash
fleet-adt4 --network silvus
```

To use a different topology file or address overlay:
```bash
fleet-adt4 --topology /path/to/topology.yaml --addresses /path/to/fleet_addresses.yaml
```

---

## CLI Mode (Non-Interactive)

`fleet-adt4` also supports command-line subcommands that run without the TUI:

```bash
# Check connectivity status of all machines
fleet-adt4 status [--role robot|base_station]

# List experiments on fleet machines
fleet-adt4 maps [--machine NAME] [--detail]

# Transfer an experiment
fleet-adt4 transfer --from SOURCE --experiment NAME [--to DST...] [--include-bags] [--relay]

# Verify file consistency across machines
fleet-adt4 verify --experiment NAME [--machine NAME]

# Monitor fleet runtime status
fleet-adt4 monitor [--machine NAME] [--watch]

# Generate/deploy Zenoh configs
fleet-adt4 zenoh [--machine NAME] [--dry-run]
```

---

## Common Workflows

### First-time fleet setup

```bash
fleet-adt4
# Press c → Config Screen
# Press F2 to add each machine (name, role, platform, IPs)
# Press F5 to save
```

### Running a mapping experiment

```bash
fleet-adt4
# Press l → Launch Screen
# Press m to apply mapping preset
# Enter experiment name (e.g., "20260405_building_a")
# Press d to preview, g to launch
```

### Running a relocalize experiment

```bash
fleet-adt4
# Press l → Launch Screen
# Press r to apply relocalize preset
# Select prior map from the list (or press p to refresh)
# Enter experiment name
# Press g to launch
```

### Checking fleet health before an experiment

```bash
fleet-adt4
# Press s to refresh connectivity
# Press o → Monitor Screen
# Press r to refresh system status
# Press n to check ROS2 node readiness
# Press F3 to verify Silvus routes
# Press z to verify Zenoh connectivity
```

### Transferring a map after an experiment

```bash
fleet-adt4
# Press b → Maps Screen
# Select the experiment
# Press t → Transfer Screen
# Select destination machines
# Press t to start
```
