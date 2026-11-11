# Fleetlink mode: bench cutover and field acceptance

Bench steps for moving one robot and the base station from **router mode** (all ROS graphs
merged over the radio by `rmw_zenohd` router-to-router links) to **fleetlink mode** (each ROS
graph stays on its own machine; only fleetlink crosses the radio). Background: the
*Fleetlink* section of [`../README.md`](../README.md) and the fleetlink README
([`../fleetlink/README.md`](../fleetlink/README.md)), sections *Router modes* and
*Generic ROS relay*.

Before touching hardware, run the multi-router harness. It runs the same topology as separate
processes on one machine, using ports 17470-17477 and ROS domains 94-96, and never touches 7447:

```bash
cd $ADT4_WS/src/awesome_dcist_t4/fleetlink
zsh -c 'source /opt/ros/jazzy/setup.zsh; source $ADT4_WS/install/setup.zsh; \
  $ADT4_ENV/spark_env/bin/python3 -m pytest -q -s tests/test_fleet_harness.py'
# a 60 s dropout (default 20 s):
FLEETLINK_STALL_S=60 zsh -c '...same...'
```

## 0. Prerequisites

* **Two machines.** Use the base station (e.g. `willow`) and one robot (or a laptop acting as
  the robot). Do not run both on one host: they would share the ROS router on 7447 and merge
  their graphs. Zenoh would also carry big payloads through shared memory, bypassing the
  network.
* Both machines are on the same commit of `awesome_dcist_t4`, and
  `dcist_launch_system/config/fleetlink/relay.yaml` is identical on both (it is installed with
  `dcist_launch_system`, so rebuild after changing it). The heartbeat compares hashes and `link_status` shows `cfg_match`.
* `install/spark_requirements.txt` is installed in `$ADT4_ENV/spark_env`, which provides
  `eclipse-zenoh==1.8.0`. Both machines have the same message packages:
  `robot_executor_msgs`, `heracles_ros_interfaces`, `dcist_launch_system_msgs`. A type-hash
  mismatch rejects messages.
* `network_diagnostics/config/topology.yaml` lists the robot with `role: robot`,
  `platform_type` and its addresses, and the base under `fleet_defaults.fleetlink_base`.
  `zenoh.fleetlink_port` is `7450` and `zenoh.fleetlink_networks` lists the networks the
  robot may reach the base on, in order of preference.
  `python -m dcist_launch_system.fleet_env --check` reports problems.
* `network_diagnostics/scripts/preflight.sh` passes its *Fleetlink* checks on both machines.
* **Robot executor.** A bag session (`spot_bag*`) has no executor, so plans are acked with
  `subs: 0` and no `action_result` comes back. For the plan → result checks the robot needs
  one of:
  * the spot sim executor: the `spot_sim_fleetlink-default` session (`spot_sim` + the
    `fleetlink` window).
  * the real Spot (`spot-default`).

## 1. Record the router-mode baseline

On the base, run `ros2 node list | sort > /tmp/router_nodes.txt`. Robot nodes such as
`/euclid/...` should appear. This is what fleetlink mode removes.

## 2. Switch the topology

1. Check that `topology.yaml` has `fleet_defaults.fleet_link: fleetlink` (the default) on
   **both** machines. To try one machine without editing,
   `export ADT4_FLEETLINK_MODE=fleetlink` before `run-adt4`.
2. Check what each machine will use:
   ```bash
   python -m dcist_launch_system.fleet_env willow    # base: ADT4_FLEETLINK_CONNECT=tcp/127.0.0.1:7450
   python -m dcist_launch_system.fleet_env euclid    # robot: tcp/<willow on each fleetlink network>:7450
   ```
   For a bench base that is not in the topology, add it to a copy of `topology.yaml` and
   `export ADT4_TOPOLOGY=<that copy>` on both machines (run-adt4 always derives the
   endpoints from the topology).
3. Redeploy the zenoh config (fleet TUI Zenoh screen or `fleet-adt4 zenoh`).
   In fleetlink mode this **clears the base router's robot endpoints**. Restart the base
   `rmw_zenohd`, then on the base check that `ss -tn state established '( dport = :7447 )'`
   shows no robot addresses.

## 3. Start the sessions

**Base** (omniplanner, heracles_state_updater, RViz and the `fleetlink_base` window):

```bash
run-adt4 base_station_heracles-default -n willow -p <prior map>
```

In the `fleetlink_base` window, pane 1 must print
`fleetlink-router: .../fleetlink_router.json5 (override: listen/endpoints=["tcp/[::]:7450"])`
and pane 2 must log `fleetlink-bridge base=willow cfg=<hash> robots=[...]`. If the panes say
"router mode", `ADT4_FLEETLINK_MODE` was not `fleetlink`.

**Robot** (`spot_sim_fleetlink-default` from §0, `spot-default`, or a bag):

```bash
run-adt4 spot_sim_fleetlink-default -n euclid -s           # sim executor
run-adt4 spot_bag-default -n euclid -s --bag <bag>        # bag: map/TF/state only
```

In the `fleetlink` window the agent must log `fleetlink relay robot=euclid platform=spot ...`.
If the base is unreachable it says `no zenoh router at [...] yet ...; retrying`, which is
harmless.

## 4. What to check

| # | check | how | pass |
|---|---|---|---|
| 1 | base graph has no robot nodes | base: `ros2 node list` | only base nodes and `/fleetlink_bridge`; nothing under `/euclid/` except topics the bridge republishes (`ros2 topic info -v /euclid/spot_executor_node/action_result` shows the bridge as the only publisher) |
| 2 | robot graph has no base nodes | robot: `ros2 node list` | no omniplanner, heracles or rviz nodes |
| 3 | relay link up | base: `ros2 topic echo /fleetlink/link_status` | `"robot": "euclid"`, `connected: true`, `cfg_match: true`, `rtt` a number |
| 4 | plan dispatch | send a goal (`adt4-sending-goals` skill) | robot executor starts; `link_status` `topics["/euclid/omniplanner_node/compiled_plan_out"]` has `acked` incremented and `subs ≥ 1` |
| 5 | `action_result` arrives | base: `ros2 topic echo /euclid/spot_executor_node/action_result` | one result per action; the planner advances |
| 6 | holding state | robot pick → `/euclid/update_holding_state` | heracles logs the update; the robot's call returns `success` |
| 7 | TF | RViz fixed frame `map` | `euclid/base_link` moves; the static `euclid/*` frames hang under it |
| 8 | RViz plan colors | RViz (`ADT4_RVIZ_CONFIG`, generated fleet config) | the plan path and markers use the robot's topology `color` |
| 9 | link status | base: `ros2 topic echo /fleetlink/link_status` | robot live, RTT and loss shown |
| 10 | dropout | unplug or turn off the robot radio for 60 s mid-plan, then restore | no process restarted; C1 state back within ~2 s; the plan is **not** re-executed; results queued during the dropout arrive (reliable, 30 tries, 1 s apart) |
| 11 | no #1876 | grep every pane log and `journalctl` | `Unable to push non droppable` never appears |

A plan published while the link is down is delivered only if it is younger than its TTL (30 s)
when the link returns. An older plan is dropped and counted under `expired`. This is on
purpose, because a stale plan must never run.

## 5. Revert to router mode

1. `fleet_defaults.fleet_link: router` (or `export ADT4_FLEETLINK_MODE=router`) on both
   machines.
2. Redeploy the zenoh config. This restores the base router's robot endpoints. Restart the
   base `rmw_zenohd`.
3. Restart the sessions. The `fleetlink_base` panes then only print "router mode", and the
   agent connects to the local `tcp/127.0.0.1:7447` with the relay off.
4. On the base, `ros2 node list` matches `/tmp/router_nodes.txt` again.

## 6. Field acceptance

A field run in fleetlink mode should meet all of these:

* no `Unable to push non droppable` in any log over 2 h;
* plan dispatch → `action_result` at the base, p95 < 3 s with 3 robots;
* a radio dropout of 60 s or more recovers without restarting any process;
* the base ROS graph has no robot nodes;
* per-robot radio bytes stay within budget (`fleetlink/config/fleetlink.yaml` budgets, plus
  relay traffic at its `rate_hz`; Silvus telemetry or `/fleetlink/link_status`);
* `/fleetlink/link_status` shows each robot's RTT and loss.
