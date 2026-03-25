# fleet-adt4 — Fleet Management Tool for ADT4

## Overview

`fleet-adt4` is a fleet management tool for the ADT4 multi-robot system. It provides both an interactive terminal UI (TUI) and a CLI for managing robot connectivity, launching experiments, browsing/transferring maps, and monitoring system status — all from a single terminal on the base station.

## What Changed

### New Files

| File | Purpose |
|------|---------|
| `dcist_launch_system/bin/fleet-adt4` | Main executable — TUI dashboard + CLI subcommands (~550 lines) |
| `dcist_launch_system/src/dcist_launch_system/fleet_helpers.py` | Shared helper functions for SSH, topology, rsync, and remote queries (~230 lines) |
| `dcist_launch_system/tests/conftest.py` | Pytest fixtures (sample topology, paths) |
| `dcist_launch_system/tests/test_fleet_helpers.py` | 40 unit + integration tests for all helper functions |

### Modified Files

| File | Change |
|------|--------|
| `network_diagnostics/config/topology.yaml` | Added `platform_id` field to each robot machine (e.g., `euclid` → `smaug`, `hamilton` → `topaz`) |

### No New Dependencies

All libraries used (`click`, `rich`, `pyyaml`, `textual`) are already dependencies in the project.

## How It Works

### Architecture

```
fleet-adt4 (bin script)
    ├── CLI mode:  click subcommands → rich table output
    └── TUI mode:  textual App → modal screens with keybindings
            │
            ▼
    fleet_helpers.py (shared logic)
        ├── load_topology()        ← reads topology.yaml
        ├── detect_network()       ← matches local IPs to subnets
        ├── resolve_machines()     ← filters machines by network/role/name
        ├── ping_host()            ← ICMP ping
        ├── ssh_cmd()              ← SSH command execution
        ├── rsync_transfer()       ← direct or relay file transfer
        ├── list_remote_experiments() ← ls adt4_output on remote machines
        ├── hash_remote_experiment()  ← md5sum map files for verification
        ├── get_remote_status()    ← tmux/ROS2/load/disk on remote machines
        └── run_parallel()         ← ThreadPoolExecutor for multi-machine ops
```

### Two Modes

**TUI mode** (run `fleet-adt4` with no subcommand):
- Interactive dashboard with keybindings: **S**tatus, **M**aps, **L**aunch, **C**onfig, **O**monitor, **Q**uit
- Modal screens for each operation (maps browser with tree view, transfer with destination selection, etc.)
- Platform IDs editable at runtime (for when laptops swap between robots)
- All SSH operations run in background threads to keep the UI responsive

**CLI mode** (run `fleet-adt4 <subcommand>`):
- Same operations as the TUI but non-interactive, for scripting or quick one-off commands
- Subcommands: `status`, `maps`, `transfer`, `verify`, `launch`, `monitor`

### Network Auto-Detection

The tool reads `topology.yaml` for machine definitions and auto-detects which network to use by matching the operator's local IP addresses against the configured subnets (MIT WiFi, Penn WiFi, Silvus). Override with `--network silvus`.

### Map Transfer Strategy

1. **Direct robot-to-robot** rsync (default) — fastest, requires inter-robot SSH keys
2. **Automatic fallback** to relay through the operator machine if direct transfer fails
3. `--relay` flag to force relay mode
4. **Auto-rename**: by default, experiments are transferred with a `prior_` prefix (e.g., `03192026_atak_test` becomes `prior_03192026_atak_test` on the destination). Use `--as NAME` to override.

### Remote Launch

Launches `run-adt4` on robots via SSH with the correct session, platform ID, output directory, and optional prior map. The `--base-station` flag also starts the base station experiment on willow.

## Usage

```bash
# Activate the environment first
source ~/environments/dcist/spark_env/bin/activate

# Interactive TUI
fleet-adt4
fleet-adt4 --network silvus

# CLI subcommands
fleet-adt4 status --ssh
fleet-adt4 maps --detail
fleet-adt4 transfer --from euclid --experiment 03192026_atak_test --to hamilton
fleet-adt4 transfer --from euclid --experiment 03192026_atak_test --as my_prior --to hamilton
fleet-adt4 verify --experiment prior_03192026_atak_test
fleet-adt4 launch --session real_spot-real_spot --experiment new_mapping --dry-run
fleet-adt4 launch --session real_spot-real_spot --experiment new_mapping --base-station
fleet-adt4 monitor --watch
```

## Test Plan

### Unit Tests (38 tests, all mocked, no network required)

Run with:
```bash
source ~/environments/dcist/spark_env/bin/activate
cd ~/dcist_ws/src/awesome_dcist_t4/dcist_launch_system
PYTHONPATH=src:$PYTHONPATH pytest tests/test_fleet_helpers.py -m "not integration" -v
```

| Category | Tests | What's Verified |
|----------|-------|-----------------|
| **load_topology** | 3 | Loads real topology.yaml, handles missing files, parses custom YAML |
| **detect_network** | 4 | Detects MIT WiFi, Silvus, unknown networks, multiple interfaces |
| **resolve_machines** | 9 | Filters by network/role/name, correct SSH user assignment, platform_id, empty results |
| **ping_host** | 4 | Success, failure, timeout, OS error |
| **ssh_cmd** | 5 | Success, failure, timeout, OS error, verifies BatchMode flag |
| **list_remote_experiments** | 4 | Parses experiment list with subdirs/sizes, empty output, SSH failure, no subdirs |
| **hash_remote_experiment** | 3 | Parses md5sum output, SSH failure, empty experiment |
| **get_remote_status** | 3 | Parses tmux/ROS2/load/disk, no sessions, SSH failure |
| **run_parallel** | 3 | Basic parallel execution, exception handling, empty input |

### Integration Tests (2 tests, require network)

Run with:
```bash
PYTHONPATH=src:$PYTHONPATH pytest tests/test_fleet_helpers.py -m integration -v
```

| Test | What's Verified |
|------|-----------------|
| `test_localhost_reachable` | `ping 127.0.0.1` succeeds |
| `test_unreachable_ip` | `ping 192.0.2.1` (TEST-NET) fails gracefully |

### Manual Test Plan (on the real fleet)

These tests require access to the robot network (WiFi or Silvus) and SSH keys configured.

#### 1. Connectivity

```bash
fleet-adt4 --network silvus status --ssh
```

- [ ] All reachable machines show green Ping and SSH
- [ ] Unreachable machines show red
- [ ] Correct IPs displayed per network
- [ ] Correct platform IDs displayed

#### 2. Map Browsing

```bash
fleet-adt4 maps --detail
```

- [ ] Lists experiments from all reachable machines
- [ ] Subdirectories (hydra, roman, etc.) shown correctly
- [ ] Sizes are reasonable
- [ ] Machines with no experiments show "no experiments"

#### 3. Map Transfer

```bash
# Create a small test directory on euclid first
ssh swarm@euclid "mkdir -p ~/adt4_output/transfer_test/hydra && echo test > ~/adt4_output/transfer_test/hydra/test.json"

# Transfer to hamilton (saved as prior_transfer_test by default)
fleet-adt4 transfer --from euclid --experiment transfer_test --to hamilton

# Transfer with custom name
fleet-adt4 transfer --from euclid --experiment transfer_test --as my_map --to hamilton

# Verify
fleet-adt4 verify --experiment prior_transfer_test
```

- [ ] Transfer completes (direct or relay)
- [ ] Default destination name is `prior_transfer_test`
- [ ] `--as` overrides the destination name
- [ ] Verify shows all files consistent
- [ ] `--subdir hydra` transfers only the hydra subfolder
- [ ] `--delete` removes extraneous files on destination

#### 4. Transfer Fallback

```bash
# Force relay mode
fleet-adt4 transfer --from euclid --experiment transfer_test --to hamilton --relay
```

- [ ] Relay transfer completes via operator machine
- [ ] Progress displayed per destination

#### 5. Launch (Dry Run)

```bash
fleet-adt4 launch --session real_spot-real_spot --experiment test_launch --dry-run
fleet-adt4 launch --session real_spot-real_spot --experiment test_launch --base-station --dry-run
```

- [ ] SSH commands printed with correct robot names, platform IDs, output dirs
- [ ] `--base-station` adds willow with base_station session
- [ ] `--prior-map` flag appears in commands when specified

#### 6. Launch (Real)

```bash
fleet-adt4 launch --session real_spot-real_spot --experiment test_launch --machine euclid
```

- [ ] tmux session starts on euclid
- [ ] Correct environment variables set (ADT4_ROBOT_NAME, ADT4_PLATFORM_ID, etc.)

#### 7. Monitor

```bash
fleet-adt4 monitor
fleet-adt4 monitor --watch
```

- [ ] Shows tmux sessions, ROS2 process count, load, disk per machine
- [ ] `--watch` refreshes every 10 seconds
- [ ] Ctrl+C exits cleanly

#### 8. TUI Interactive Mode

```bash
fleet-adt4 --network silvus
```

- [ ] Dashboard renders with fleet status table
- [ ] **S** refreshes status
- [ ] **M** opens map browser with tree view, experiments selectable
- [ ] **T** from map browser opens transfer screen with destination checkboxes
- [ ] **V** from map browser shows hash comparison table
- [ ] **C** opens config editor, platform_id changes persist in session
- [ ] **L** opens launch form with session/experiment inputs
- [ ] **D** in launch screen shows dry-run commands
- [ ] **O** opens monitor with refresh on **R**
- [ ] **Escape** returns to main screen from any modal
- [ ] **Q** exits

#### 9. Config Override

```bash
# In TUI, press C, change euclid's platform_id from smaug to topaz
# Then press L and verify the launch command uses the new platform_id
```

- [ ] Changed platform_id reflected in launch commands
- [ ] Original topology.yaml unchanged

### Cleanup After Manual Tests

```bash
ssh swarm@euclid "rm -rf ~/adt4_output/transfer_test"
ssh swarm@hamilton "rm -rf ~/adt4_output/transfer_test"
```
