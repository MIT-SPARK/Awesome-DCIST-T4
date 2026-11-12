---
name: deploy-adt4
description: Deploy an Awesome-DCIST-T4 branch to robots and the base station - check out the branch in each machine's colcon workspace, update all submodules recursively, and build. Use when the user asks to deploy, update or roll out a branch on the robots / base station / fleet, or to check how a deploy went.
---

# Deploying ADT4 to the fleet

A deploy on one machine is: go into its workspace (`$ADT4_WS`, usually `~/dcist_ws`), check
out the branch in `src/awesome_dcist_t4`, update every submodule recursively, and build.
`deploy_on_machine.sh` does exactly that on the machine; `fleet_deploy.sh` runs it on several
machines over SSH.

```bash
SK=$ADT4_WS/src/awesome_dcist_t4/.claude/skills/deploy-adt4
$SK/fleet_deploy.sh start develop-nov11 user@base user@robot1 user@robot2
$SK/fleet_deploy.sh status user@base user@robot1 user@robot2
$SK/fleet_deploy.sh wait   user@base user@robot1 user@robot2   # blocks until all builds end
```

`start` copies `deploy_on_machine.sh` to each machine and runs it in a tmux session
`adt4_deploy` (log `~/adt4_deploy_<date>.log`). Watch one with `ssh <host> -t tmux attach -t adt4_deploy`.

## What deploy_on_machine.sh does

1. Saves the current state: `~/adt4_predeploy_<date>.txt` (branch, commit, submodule pins,
   local changes) and a branch `predeploy-<date>` at the old commit.
2. Fetches the branch from MIT-SPARK/Awesome-DCIST-T4 (or the repo URL given as the second
   argument) and checks it out (`git checkout -f -B <branch>`).
3. Moves aside any submodule path that is a plain folder (code that used to live inside the
   repo, e.g. `fleetlink/`, `adt4_console/` before they became submodules) to
   `~/adt4_predeploy_<date>_intree/`. Left in place, such a folder blocks the submodule, and git
   commands run inside it act on the main repo instead.
4. `git submodule sync --recursive && git submodule update --init --recursive`, then checks
   that every submodule is at its pinned commit and that the main repo is still at the branch.
   Commits that only exist on a fork (e.g. harelb/...) are served through the upstream repo, so
   they fetch like any other pin. Submodules with `update = none` (e.g. `agentic_navigation` on
   machines without access) are skipped.
5. `colcon build --continue-on-error`, printing the summary and `BUILD_EXIT`.

It stops before building on `FETCH_FAIL`, `CHECKOUT_FAIL`, `SUBMODULES_FAIL` (a `MISMATCH`
line names the submodule) or `REPO_MOVED`.

## Before deploying

- **Ask before stopping anything.** `start` skips a machine with a running `adt4_system`
  session or bag recorder. Stop a recording with Ctrl-C in its `recording` window (see the
  `recording-adt4` skill) so the bag gets its `metadata.yaml`, then `tmux kill-session -t adt4_system`.
- Check disk space (`df -h ~`); a full build needs a few GB.
- Fleet tools read machine addresses from the private overlay
  (`~/.config/adt4/fleet_addresses/fleet_addresses.yaml`, repo robustrobotics/adt4_fleet_addresses).
  Make sure each machine has it.

## After deploying

- `fleet_deploy.sh status` should show `submodules: all at their pins` and `BUILD_EXIT=0` with a
  colcon `Summary:` line and no `Failed` packages on every machine.
- Python environments are separate from the build. Re-run `install/python_setup.bash` when a
  deploy adds Python dependencies (on the base station add `--openset-query` for the SAM3
  object search); the web console's UI is built by that script and needs Node >= 20.19.
- Deploying does not start anything; launch sessions with `run-adt4` (see the `run-adt4` skill).

## Going back

```bash
cd $ADT4_WS/src/awesome_dcist_t4 && git checkout -f predeploy-<date> && git submodule update --init --recursive
cd $ADT4_WS && colcon build --continue-on-error
```
Folders moved aside are in `~/adt4_predeploy_<date>_intree/`.
