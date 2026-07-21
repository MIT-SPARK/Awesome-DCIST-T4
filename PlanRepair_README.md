# Plan Repair / Plan Interruption (Omniplanner + ADT4)

This adds **plan repair**, **plan interruption**, and **runtime constraints**
(forbidden POIs) to the ADT4 + omniplanner stack. All plan-repair code lives
inside `omniplanner_ros` (no separate ROS package).

While a plan is executing you can publish a new PDDL goal. The system:

1. Pauses the robot immediately.
2. Checks whether the current symbolic plan already covers the new goal
   (visits all requested POIs and avoids any forbidden ones).
3. If yes → resume the current plan (no replanning).
   If no → replan from the current world state; the executor preempts the old
   plan (hard stop) and executes the new one.

Same behavior in RViz/fake-Spot simulation and on real Spot.

---

## 1. Components

### `omniplanner_ros` (this package)

| File | Role |
| --- | --- |
| `omniplanner_ros/src/omniplanner_ros/last_pddl_plans.py` | In-process cache `{robot_name → PddlPlan}`. |
| `omniplanner_ros/src/omniplanner_ros/plan_utils.py` | Extract `visited-{place,object,poi}` from goal strings, extract POIs visited by a plan, parse `forbidden-poi` constraints, coverage check. |
| `omniplanner_ros/src/omniplanner_ros/omniplanner_node.py` | **Unchanged.** Vanilla omniplanner node — no plan-repair behavior at runtime. |
| `omniplanner_ros/src/omniplanner_ros/pddl_planner_ros.py` | Adds three `set_last_plan(...)` hooks inside `compile_pddl_plan` (single-robot) and `compile_multirobot_pddl_plan` (multi-robot) so the cache is populated whenever a PDDL plan is compiled. Side-effect-free: just a dict write per planning call. |
| `omniplanner_ros/src/omniplanner_ros/omniplanner_repair_node.py` | `OmniPlannerRepairRos`, subclass of `OmniPlannerRos`. Adds `~/constraints` topic, per-message constraints from `PddlGoalMsg.constraints`, per-robot `/<robot>/omniplanner_node/plan_visited_pois` publisher, traceback logging. Activated only via `launch_omniplanner_repair:=true`. |
| `omniplanner_ros/src/omniplanner_ros/goal_manager_node.py` | Front-end for user PDDL goals. Pauses the executor, checks coverage against the cached visited POIs, then resumes or forwards the goal to omniplanner. |
| `omniplanner_ros/setup.py` | Adds two console_scripts: `omniplanner_repair_node`, `goal_manager_node`. The original `omniplanner_node` and `pddl_plan_rviz_viz` entries are unchanged. |

### `omniplanner` core changes

| File | Change |
| --- | --- |
| `omniplanner_msgs/msg/PddlGoalMsg.msg` | Added `string constraints` field (additive, backwards compatible). |
| `dsg_pddl/domains/RegionObjectRearrangementDomain_MultiRobot_FD_Explore_Constraints.pddl` | New domain variant that respects `(forbidden-poi)` / `(forbidden-edge)` in `goto-poi` preconditions. |
| `dsg_pddl/dsg_pddl_grounding_multirobot.py` | Injects runtime constraints into the PDDL init. |
| `dsg_pddl/dsg_pddl_planning.py` | Optional env-var override for the Fast Downward search alias / time limit. |

### `spot_tools` submodule (executor preemption)

| File | Change |
| --- | --- |
| `spot_tools/src/spot_executor/spot_executor.py` | New `set_paused()`; `terminate_sequence()` does a best-effort hard stop (`set_vel(0,0)` for FakeSpot, `sit()` for real Spot). Per-action pause loop. |
| `spot_tools/src/spot_skills/navigation_utils.py` | `follow_trajectory_continuous` takes `cancel_cb` / `pause_cb` so trajectories can be preempted mid-segment. |
| `spot_tools/src/spot_executor/fake_spot.py` | Interpolated motion (10 Hz steps) instead of teleport so preemption is visible in sim. |
| `spot_tools_ros/src/spot_tools_ros/spot_executor_ros.py` | Tracks `_current_plan_id`; new plan with a different id preempts the old. New `~/pause`, `~/resume`, `~/stop` `Bool` topics. |

### `heracles_agents` submodule (only for the `_heracles_repair` tmux variant)

| File | Change |
| --- | --- |
| `examples/chatdsg_repair/` | Repair-aware chat agent example (`agent_config.yaml`, `agent_prompt.yaml`, `chatdsg.py`). |
| `examples/prompts/common/pddl_domain_description_with_constraints.yaml` | NL description of the constrained domain. |
| `examples/prompts/common/pddl_in_context_examples_with_constraints.yaml` | In-context examples showing how to emit a goal + constraints. |
| `src/heracles_agents/tools/pddl_repair_tool.py` | Agent tool that talks to the goal manager. |

### `awesome_dcist_t4` (outer repo)

| File | Change |
| --- | --- |
| `dcist_launch_system/launch/master.launch.yaml` | New flag `launch_omniplanner_repair`. When true, launches `omniplanner_repair_node` + `goal_manager_node` from **`omniplanner_ros`**. |
| `dcist_launch_system/config/spot_prior_dsg/omniplanner_plugins.yaml` | Switched `multi_robot_pddl` domain to `..._Constraints` variant. |

---

## 2. Data Flow

```
user PDDL goal                       symbolic-plan POIs
     │                                       ▲
     ▼                                       │
/<robot>/commanded_pddl_goal                 │
[PddlGoalMsg]                                │
     │                                       │
     ▼                                       │
┌──────────────┐  pause/resume   ┌───────────────────────┐
│ goal_manager │ ──────────────▶ │ spot_executor (Spot)  │
└──────────────┘                 └───────────────────────┘
     │ (only if cached plan does NOT cover)  ▲
     ▼                                       │ ActionSequenceMsg (plan_id)
/<robot>/omniplanner_node/multi_robot_pddl/pddl_goal
[PddlGoalMsg, with constraints]              │
     │                                       │
     ▼                                       │
┌─────────────────────────────────┐          │
│ omniplanner_repair_node         │ ─────────┘
│  - merges per-msg + ~/constraints
│  - publishes plan_visited_pois  │ ───────▶ /<robot>/omniplanner_node/plan_visited_pois [String JSON]
└─────────────────────────────────┘                                          │
              ▲                                                              ▼
              │ set_last_plan(...)                                   goal_manager cache
              │
       pddl_planner_ros.py
       (hooks inside compile_pddl_plan / compile_multirobot_pddl_plan)
```

Key topics:

- **User input:** `/<robot>/commanded_pddl_goal` — `omniplanner_msgs/PddlGoalMsg`
- **Goal manager → omniplanner:** `/<robot>/omniplanner_node/multi_robot_pddl/pddl_goal` — same type
- **Repair node → goal manager:** `/<robot>/omniplanner_node/plan_visited_pois` — `std_msgs/String` (JSON list of POI ids)
- **Persistent constraints:** `/<omniplanner_node_ns>/constraints` — `std_msgs/String` (JSON list of facts). Send `[]` to clear.
- **Executor control:** `/<robot>/spot_executor_node/{pause,resume,stop}` — `std_msgs/Bool`

`PddlGoalMsg.constraints` is a JSON-encoded list, e.g.
`'[["forbidden-poi","p123"]]'`. The repair node merges it with the persistent
`~/constraints` for that single planning call and adds it to the persistent set.

---

## 3. Behavior in Detail

### Goal manager (`omniplanner_ros.goal_manager_node`)

On every `PddlGoalMsg` on `/<robot>/commanded_pddl_goal`:

1. Publish `Bool(True)` on `/<robot>/spot_executor_node/pause` — robot pauses
   visibly before any decision is made.
2. Extract requested POIs from `pddl_goal` (`visited-place|object|poi`) and
   forbidden POIs from `constraints`.
3. Look up the latest visited POIs cached for the robot (received over
   `plan_visited_pois`).
4. If the cache is non-empty **and** `requested ⊆ cached` **and**
   `forbidden ∩ cached = ∅`:
   - publish `Bool(True)` on `/<robot>/spot_executor_node/resume`.
   - log "current plan already visits ...".
   - **Do not** forward to omniplanner.
5. Otherwise forward the original `PddlGoalMsg` (with constraints) to omniplanner.

### Repair node (`omniplanner_ros.omniplanner_repair_node`)

`OmniPlannerRepairRos` extends `OmniPlannerRos` and overrides `register_plugin`:

- Merges `PddlGoalMsg.constraints` with the persistent `active_constraints`
  list and attaches them to the `plan_request` before planning.
- After `compile_plan` completes, calls `get_last_plan(robot_name)` (cache
  populated by the `set_last_plan` hooks in `pddl_planner_ros.py`) and
  publishes the set of POIs the plan visits
  (`extract_visited_pois_from_plan`) on
  `/<robot>/omniplanner_node/plan_visited_pois`.
- Logs the full symbolic plan as `=== Symbolic plan for <robot> ===` followed
  by `[i] (action, ...)` one per line — useful for debugging "why does the
  replan look weird?".

### Plan cache hooks (`pddl_planner_ros.py`)

Side-effect-free additions inside the compile functions:

```python
# compile_pddl_plan (single-robot)
plan = contextualized_plan.value
set_last_plan(robot_name, plan)        # <-- new
...

# compile_multirobot_pddl_plan
for rn, p in plan_per_robot.items():   # <-- new
    set_last_plan(rn, p)
```

These run on every PDDL compile, including when only the vanilla
`omniplanner_node` is launched. They just write to a module-level dict and
return None; they cannot raise (string keys, simple dict).

### Executor preemption (`spot_executor_ros.py` + `spot_executor.py`)

- `process_action_sequence` tracks `_current_plan_id`. If a new
  `ActionSequenceMsg` arrives with a **different** `plan_id` while a thread is
  still running, it calls `terminate_sequence()` and starts a new thread.
- `terminate_sequence()` sets `keep_going = False`, sets
  `break_out_of_waiting_loop`, attempts a hard stop (`set_vel(0,0)` for
  FakeSpot, `sit()` for real Spot), and waits for the previous thread to
  finish.
- `set_paused(True)` causes the per-action loop to spin in place and
  immediately commands the robot to stop / sit. `set_paused(False)` resumes.
- `follow_trajectory_continuous` polls `cancel_cb` and `pause_cb` at ~10 Hz so
  preemption / pause works mid-trajectory.

---

## 4. How to Run It

```bash
cd ~/colcon_ws
colcon build --packages-select omniplanner_ros
source install/setup.bash
```

Launch ADT4 with repair mode:

```bash
ros2 launch dcist_launch_system master.launch.yaml \
    launch_omniplanner_repair:=true \
    robot_name:=hilbert \
    ...
```

Or use one of the tmux configs:

- `tmux/autogenerated/spot_prior_dsg_repair-spot_prior_dsg.yaml` — plain repair mode.
- `tmux/autogenerated/spot_prior_dsg_heracles_repair-spot_prior_dsg.yaml` —
  repair + the chatdsg_repair agent. **Requires** these env vars:

  ```bash
  export ADT4_HERACLES_AGENTS_PATH=$ADT4_WS/src/awesome_dcist_t4/heracles_agents
  export ADT4_HERACLES_IP=...
  export ADT4_HERACLES_PORT=...
  export ADT4_NEO4J_USERNAME=...
  export ADT4_NEO4J_PASSWORD=...
  export ADT4_OPENAI_API_KEY=...
  ```

Send a goal:

```bash
ros2 topic pub -1 /hilbert/commanded_pddl_goal \
  omniplanner_msgs/msg/PddlGoalMsg \
  "{robot_id: 'hilbert', pddl_goal: '(visited-place p5041)', constraints: ''}"
```

Send a goal with a runtime constraint:

```bash
ros2 topic pub -1 /hilbert/commanded_pddl_goal \
  omniplanner_msgs/msg/PddlGoalMsg \
  "{robot_id: 'hilbert',
    pddl_goal: '(and (visited-place p5041))',
    constraints: '[[\"forbidden-poi\",\"p1735\"]]'}"
```

Clear all persistent constraints:

```bash
ros2 topic pub -1 /hilbert/omniplanner_node/constraints std_msgs/msg/String "{data: '[]'}"
```

Expected behavior over time:

- **First goal:** goal manager forwards it, omniplanner plans, executor starts,
  `=== Symbolic plan for hilbert ===` is logged, `plan_visited_pois` is
  published, goal_manager logs `updated plan cache: N visited POIs`.
- **Subsequent goal already covered by the current plan:** goal manager pauses,
  detects coverage, resumes. No replan, no preemption.
- **Subsequent goal not covered (or violates a `forbidden-poi`):** goal manager
  pauses then forwards. Omniplanner replans from the current DSG/TF. New
  `ActionSequenceMsg` arrives with a new `plan_id`. Executor terminates the
  old plan (hard stop) and starts the new one.

---

## 5. Pull Request / Merge Plan

The work spans **four git repositories**: the outer workspace
`awesome_dcist_t4` and three submodules — `omniplanner`, `spot_tools`,
and `heracles_agents` (only needed for the heracles-repair tmux variant).
One PR per repo.

### 5.1 `omniplanner` submodule

- Local branch: `multi_robot_plan_repair` (pushed to origin).
- `origin/main` is only ~1 commit ahead of the branch point.
- Final state of the changes in this PR:

  **In `omniplanner_msgs`:**
  - `omniplanner_msgs/msg/PddlGoalMsg.msg` — new `string constraints` field
    (additive, backwards compatible).

  **In `dsg_pddl`:**
  - `dsg_pddl/domains/RegionObjectRearrangementDomain_MultiRobot_FD_Explore_Constraints.pddl` — new constrained variant of the multi-robot domain.
  - `dsg_pddl/dsg_pddl_grounding_multirobot.py` — injects runtime
    `forbidden-poi` / `forbidden-edge` facts into the PDDL init.
  - `dsg_pddl/dsg_pddl_planning.py` — `ADT4_FD_*` env-var overrides for
    Fast Downward search alias / time limit. (Optional; can be split off.)

  **In `omniplanner_ros`:**
  - `omniplanner_ros/src/omniplanner_ros/last_pddl_plans.py` — new (~15 lines, just a dict cache).
  - `omniplanner_ros/src/omniplanner_ros/plan_utils.py` — new (~50 lines, pure helpers).
  - `omniplanner_ros/src/omniplanner_ros/goal_manager_node.py` — new ROS node, only runs if launched via the new flag.
  - `omniplanner_ros/src/omniplanner_ros/omniplanner_repair_node.py` — new subclass of `OmniPlannerRos`; only runs if launched via the new flag.
  - `omniplanner_ros/src/omniplanner_ros/pddl_planner_ros.py` — adds `from .last_pddl_plans import set_last_plan` plus 4 lines of `set_last_plan(...)` calls inside the existing compile functions. Side-effect-free: just a dict write per planning call. **The only modification to existing code.**
  - `omniplanner_ros/setup.py` — adds two `console_scripts` entries (`omniplanner_repair_node`, `goal_manager_node`). The original `omniplanner_node` and `pddl_plan_rviz_viz` entries are unchanged.

- **Safety claim for reviewers:** vanilla `ros2 run omniplanner_ros omniplanner_node` behavior is byte-for-byte unchanged. The new entry points are opt-in. The only modification to existing code paths is the 4-line `set_last_plan` hook in `pddl_planner_ros.py`, which writes to a module-level dict and is never read unless `omniplanner_repair_node` is also running.

- Untracked junk to ignore: `omniplanner/examples/convoy_*.py` and the
  `RegionObjectRearrangemen` directory (typo). Decide before opening the PR.

**Action:** commit the kept changes above, push, open PR
`multi_robot_plan_repair → main`. **Expected conflicts: very low.**

### 5.2 `spot_tools` submodule

- HEAD is **detached** at `b40d8c0`. Create a feature branch before pushing.
- `origin/main` is 1 commit ahead (`0700fd8 Feature/debugging lease handoff`)
  → check for conflicts in `spot_executor.py` and `navigation_utils.py`.
- All changes are uncommitted: `set_paused`, hard-stop in
  `terminate_sequence`, `cancel_cb` / `pause_cb` in
  `follow_trajectory_continuous`, fake-spot interpolation, ROS
  `~/pause` `~/resume` `~/stop` topics, plan-id preemption.
- `MAX_LINEAR_VEL=0.25` / `MAX_ROTATION_VEL=0.25` in `navigation_utils.py` is
  **demo tuning**, not a plan-repair feature — drop or gate behind a param
  before merging.

**Action:**
```bash
cd src/awesome_dcist_t4/spot_tools
git switch -c feature/plan-repair-preemption
# commit by topic:
#   - executor preemption + plan-id tracking
#   - pause/resume/stop topics + set_paused / terminate hard-stop
#   - fake-spot interpolation
#   - cancel_cb / pause_cb plumbing in follow_trajectory_continuous
git push -u origin feature/plan-repair-preemption
```

**Expected conflicts: low–medium**, mostly around `spot_executor.py` if the
lease-handoff PR also touched the action sequence loop.

### 5.3 `heracles_agents` submodule (only for the `_heracles_repair` tmux variant)

- HEAD is detached at the tip of `master`. Create a feature branch before pushing.
- Untracked plan-repair contributions to commit:
  - `examples/chatdsg_repair/` — repair-aware chat agent example.
  - `examples/prompts/common/pddl_domain_description_with_constraints.yaml`.
  - `examples/prompts/common/pddl_in_context_examples_with_constraints.yaml`.
  - `src/heracles_agents/tools/pddl_repair_tool.py`.

**Action:**

```bash
cd src/awesome_dcist_t4/heracles_agents
git switch -c feature/plan-repair
git add examples/chatdsg_repair/ \
        examples/prompts/common/pddl_domain_description_with_constraints.yaml \
        examples/prompts/common/pddl_in_context_examples_with_constraints.yaml \
        src/heracles_agents/tools/pddl_repair_tool.py
git commit -m "Add chatdsg_repair example + constraint-aware prompts + pddl_repair_tool"
git push -u origin feature/plan-repair
```

Open PR `feature/plan-repair → master`. Expected conflicts: very low.

### 5.4 `awesome_dcist_t4` (outer repo)

- Branch: `feature/multi-robot-planning`.
- Modifications to commit:
  - `dcist_launch_system/launch/master.launch.yaml` — adds the
    `launch_omniplanner_repair` flag + the two `pyenv_node` blocks (pointing
    at `pkg: omniplanner_ros`).
  - `dcist_launch_system/config/spot_prior_dsg/omniplanner_plugins.yaml` —
    switches `multi_robot_pddl` to `..._Constraints` domain.
- **Untracked tmux yamls** (hand-edited):
  - `tmux/autogenerated/spot_prior_dsg_repair-spot_prior_dsg.yaml`
  - `tmux/autogenerated/spot_prior_dsg_heracles_repair-spot_prior_dsg.yaml`

  Should NOT be committed as-is. Generate them from
  `config_generation/experiment_manifest.yaml`.

**Action:**

1. Add a launch component `config_generation/launch_components/planning_repair.yaml`
   (copy of `planning.yaml`, change `launch_omniplanner:=true` →
   `launch_omniplanner_repair:=true`). If the heracles variant needs
   `launch_scene_graph_publisher` instead of `launch_heracles_publisher`,
   also add `config_generation/launch_components/heracles_prior_dsg_repair.yaml`.
2. Add new entries in `config_generation/experiment_manifest.yaml`:

   ```yaml
   launch_configs:
     plan_prior_repair:               [main, prior_dsg, planning_repair]
     plan_prior_heracles_repair:      [main, heracles_prior_dsg_repair, planning_repair]
     spot_prior_dsg_repair:           [plan_prior_repair, sim_spot]
     spot_prior_dsg_heracles_repair:  [plan_prior_heracles_repair, sim_spot]

   experiments:
     spot_prior_dsg_repair:
       launch_config: spot_prior_dsg_repair
       config: [spot_prior_dsg]
     spot_prior_dsg_heracles_repair:
       launch_config: spot_prior_dsg_heracles_repair
       config: [spot_prior_dsg]
   ```

3. Run `./scripts/generate_configs.sh`, then `./scripts/check_configs.sh` to
   verify regeneration is idempotent.
4. Wait until the three submodule PRs merge, bump the submodule SHAs, commit
   manifest + regenerated tmux yamls + launch flag + plugins yaml, open the
   outer PR.

**Expected conflicts: very low** — submodule SHA bumps + launch flag + 1 YAML line.

### 5.5 Risk summary

| Repo | Expected conflicts | Notes |
| --- | --- | --- |
| omniplanner | very low | only ~1 commit on main since branch point; only modification to existing code is a 4-line dict-write in `pddl_planner_ros.py` |
| spot_tools | low–medium | overlap likely with #31 (lease handoff); rebase to verify |
| heracles_agents | very low | all four items are new files |
| awesome_dcist_t4 | very low | submodule SHAs + launch flag + 1 YAML line |

### 5.6 Step-by-step merge order

1. **`omniplanner` PR.** Merge to `main`. Note the new SHA.
2. **`spot_tools` PR.** Merge to `main`. Note the new SHA.
3. **`heracles_agents` PR.** Merge to `master`. Note the new SHA.
4. **`awesome_dcist_t4` PR last.** Bumps the three submodule pointers and
   ships the launch + manifest changes.

If you need to ship `awesome_dcist_t4` before the submodule PRs land (e.g.
for a demo), point the submodule pointers at the feature-branch SHAs
temporarily and rebase later.

---

## 6. Why the merged layout (vs. a separate `plan_repair_ros` package)

An earlier iteration of this work lived in a sibling package
(`plan_repair_ros`) that depended on `omniplanner_ros`. That layout was
folded back in for these reasons:

- **Tighter integration.** With the cache hooks living in
  `pddl_planner_ros.py`, the repair node only needs `get_last_plan(...)` —
  no need to walk `full_planning_pipeline`'s return structure or maintain a
  separate `_peel_symbolic` helper.
- **One fewer ROS package** to build, install, and document.
- **Zero impact on vanilla users.** The only modification to existing code is
  4 lines that write to a dict no one reads unless the repair node is also
  running.
- **Single PR for the planning side.** Reviewers see one omniplanner PR
  instead of "omniplanner change + new package depending on it."

The cost — plan-repair source now lives inside an upstream-controlled repo —
is acceptable because `omniplanner` and ADT4 share maintainers (MIT-SPARK).
If that ever changes, splitting back out is mechanical: move the four files
into a new `plan_repair_ros`, replace the `set_last_plan` hooks with a
post-pipeline cache walk in the repair node, and update the launch file.
