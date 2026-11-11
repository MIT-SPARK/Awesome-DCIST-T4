---
name: recording-adt4
description: Use when recording rosbags on ADT4 robots or the base station — starting/stopping a `run-adt4 -r` style recording, adding a recording to a session that is already running, recording a whole fleet at once, or checking that a recording finished cleanly.
---

# Recording ADT4 runs

## How `run-adt4 -r` records

`run-adt4 -r [NAME]` adds a tmux window **`recording`** to the session that runs

```
ros2 run dcist_launch_system record_topics <ADT4_OUTPUT_DIR>/recorded_data dcist_launch_system/recording/<NAME>_topics.txt
```

- `NAME` defaults to the platform type (`spot`, `phoenix`); the base station uses `base_station`.
  Topic files: `dcist_launch_system/recording/{spot,phoenix,base_station}_topics.txt`.
- `record_topics` (`dcist_launch_system/src/dcist_launch_system/record_topics.py`) expands `${ADT4_ROBOT_NAME}`
  etc. in each line and runs `ros2 bag record -o <out> --topics ...`.
- The bag goes in `recorded_data/` inside the session's output directory (the `-o` of run-adt4).
  `ros2 bag record -o` refuses an existing directory: one recording per output dir.

Starting a session with recording from the beginning: add `-r` to the run-adt4 command, e.g.
`run-adt4 -n euclid -c smaug -o ~/adt4_output/run1 -p <prior> -r -y spot_relocalize-relocalize`.

## Recording a session that is already running

Open the same window by hand inside the running `adt4_system` session (it inherits the session's
`ADT4_ROBOT_NAME` / `ADT4_WS`):

```
O=~/adt4_output/<running output dir>; T=~/dcist_ws/src/awesome_dcist_t4/dcist_launch_system/recording/<spot|phoenix|base_station>_topics.txt
tmux new-window -d -t adt4_system -n recording "zsh -ic 'source \${ADT4_WS}/install/setup.zsh && ros2 run dcist_launch_system record_topics $O/recorded_data $T; exec zsh'"
```

Find the running output dir from the session: `tmux show-environment -t adt4_system ADT4_OUTPUT_DIR`.

## Whole fleet at once

`fleet_record_start.sh [machines.txt]` / `fleet_record_stop.sh [machines.txt]` in this skill directory.
`machines.txt` lines: `<ssh target> <output dir name under ~/adt4_output> <topic file prefix>` (see
`machines.example.txt`; blank lines and `#` comments are skipped). Start refuses a machine whose `recorded_data` already exists or that already has a
`recording` window. Stop sends Ctrl-C to the `recording` window (so ros2 bag writes `metadata.yaml`), waits
for the recorder to exit and reports `metadata=ok` and the bag size.

## Checks and traps

- Before starting: the session is running, `recorded_data` does not exist yet, and the disk has room —
  Spot topics include ZED RGB + depth and the Livox lidar (tens of MB/s; check `df -h ~`).
- Stop with **Ctrl-C** in the `recording` window, never `tmux kill-session` first: a killed recorder leaves
  the bag without `metadata.yaml` (recover with `ros2 bag reindex <dir>`).
- Killing the whole session (`tmux kill-session -t adt4_system`) also kills the recorder: stop the
  recording first.
- Check a finished bag: `ros2 bag info <out>/recorded_data`.
- `pgrep -f "ros2 bag record"` matches your own ssh command line; match `'bin/ros2 bag recor[d]'`.
