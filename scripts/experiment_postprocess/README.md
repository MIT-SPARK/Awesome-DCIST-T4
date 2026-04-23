# Experiment Post-Processing Pipeline

Scripts for turning raw West Point 2026 MCAP bags into a structured experiment
CSV, overhead GPS trajectory plots, and FPV videos.

## Prerequisites

Run every script from a terminal with the ROS 2 Jazzy workspace and the
`spark_env` virtualenv active:

```bash
source /opt/ros/jazzy/setup.zsh
source ~/dcist_ws/install/setup.zsh
source ~/environments/dcist/spark_env/bin/activate
```

`ffmpeg` must be on PATH for video generation.
Additional Python packages: `contextily`, `pandas` (`pip install` into `spark_env`).

## Scripts

### `build_experiment_master_list.py` — build the CSV

Walks the data root, finds every `compiled_plan_out` plan publication across
all bags, groups near-simultaneous multi-robot plans into experiments, and
writes a merged `experiments.csv`.

```bash
python build_experiment_master_list.py \
    --root /data/dcist/west_point_2026 \
    --output /home/harel/data/west_point_2026/processed/experiments.csv
```

Key options:

| flag | default | meaning |
|---|---|---|
| `--stop-speed` | `0.1` m/s | velocity threshold for "robot stopped" |
| `--stop-window` | `120` s | how long below threshold counts as a stop |
| `--multiagent-window` | `30` s | max gap between two robots' plans to group them |
| `--force` | off | skip merge — `language_command`/`notes` will be lost |

The CSV is **idempotent**: re-running preserves `language_command` and `notes`
columns that were filled in by hand, matching rows on
`(experiment_set, start_time_unix, robots)`.

---

### `visualize_experiment.py` — overhead trajectories + FPV videos

Reads `experiments.csv`, then for each selected experiment generates:

- `<output_root>/<experiment_id>/trajectory.png` — satellite basemap with
  each robot's GPS track, start/end markers, and (optionally) candidate end
  times as per-candidate PNGs named `trajectory_cand_NN.png`.
- `<output_root>/<experiment_id>/<robot>/rgb.mp4` — FPV colour video.
- `<output_root>/<experiment_id>/<robot>/depth.mp4` — colorized depth video
  (TURBO colormap, 0.3–10 m).

#### Trajectory plotting

GPS is read from `/<robot>/fix` (NavSatFix) via the MCAP chunk index so only
the experiment window is decompressed. Points are projected to Web Mercator
and overlaid on an Esri WorldImagery tile fetched by `contextily`. One
coloured line per robot; filled circle = start, filled square = end.

Candidate end times (from stop detection) are plotted as extra PNGs so you
can pick the correct cut-point in the review app.

```bash
# Single experiment
python visualize_experiment.py \
    --master /home/harel/data/west_point_2026/processed/experiments.csv \
    --experiment-id april_17_alpha__exp_0001 \
    --output-root /home/harel/data/west_point_2026/viz_out

# All experiments, trajectories only (fast)
python visualize_experiment.py \
    --master /home/harel/data/west_point_2026/processed/experiments.csv \
    --output-root /home/harel/data/west_point_2026/viz_out \
    --all --skip-video

# All experiments, videos only (slow)
python visualize_experiment.py ... --all --skip-trajectory
```

Key options:

| flag | default | meaning |
|---|---|---|
| `--experiment-id` | — | process one experiment by ID |
| `--all` | off | process every row in the CSV |
| `--output-root` | `viz_out/` | where to write outputs |
| `--rgb-fps` / `--depth-fps` | `10` | output video frame rate |
| `--no-basemap` | off | skip satellite tile (works offline) |
| `--skip-trajectory` | off | skip trajectory PNG |
| `--skip-video` | off | skip FPV MP4 generation |

---

### `dump_plans.py` — write plan text files

For each experiment, finds the `compiled_plan_out` message nearest to the
experiment start time and writes a human-readable `plan.txt` with every
action and all waypoints.

```bash
python dump_plans.py \
    --master /home/harel/data/west_point_2026/processed/experiments.csv \
    --viz-root /home/harel/data/west_point_2026/viz_out
```

---

### `review_experiments.py` — Streamlit review app

Browse trajectories, FPV videos, and metadata; edit start/end times and fill
in language commands.

```bash
streamlit run review_experiments.py -- \
    --master /home/harel/data/west_point_2026/processed/experiments.csv \
    --viz-root /home/harel/data/west_point_2026/viz_out
```

---

### `run_overnight.sh` — full pipeline runner

Runs all three steps unattended: CSV build → trajectory PNGs → FPV videos.

```bash
zsh run_overnight.sh
```

Paths are hard-coded at the top of the script. Extra args (e.g.
`--stop-speed 0.2`) are forwarded to `build_experiment_master_list.py`.

---

### `recover_mcap.py` — recover truncated bags

Copies all intact chunks from a truncated MCAP into a new file using
`NonSeekingReader` (no index required). The original is renamed
`*_original_truncated.mcap`.

```bash
python recover_mcap.py \
    --input /path/to/truncated_bag_0.mcap \
    --output /path/to/truncated_bag_0_recovered.mcap
```

---

## Data layout expected on disk

```
<root>/
  <session_name>_<robot>/          # one folder per robot per session
    recorded_data/
      recorded_data_0.mcap
    recorded_data_2/               # optional second bag in same session
      recorded_data_2_0.mcap
    spot_executor/
      lease_log.txt                # CSV: time,event
```

Sessions that share a name prefix (e.g. `april_16_alpha_euclid` and
`april_16_alpha_hamilton`) are automatically paired into multi-robot sessions
by `build_experiment_master_list.py`.
