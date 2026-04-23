#!/usr/bin/env zsh
# Full overnight pipeline:
#   1. Build master experiment CSV
#   2. Generate trajectory PNGs for all experiments
#   3. Generate RGB + depth MP4s for all experiments
#
# Run from a terminal where the dcist_ws is already sourced, or run as:
#   source /opt/ros/jazzy/setup.zsh
#   source /home/harel/dcist_ws/install/setup.zsh
#   source /home/harel/environments/dcist/spark_env/bin/activate
#   zsh run_overnight.sh

set -eu

SCRIPT_DIR="${0:a:h}"
MASTER="/home/harel/data/west_point_2026/processed/experiments.csv"
VIZ_ROOT="/home/harel/data/west_point_2026/viz_out"
DATA_ROOT="/data/dcist/west_point_2026"
PYTHON="/home/harel/environments/dcist/spark_env/bin/python"

# Disable strict mode while sourcing — setup.zsh accesses unset vars internally.
set +eu
source /opt/ros/jazzy/setup.zsh
source /home/harel/dcist_ws/install/setup.zsh
source /home/harel/environments/dcist/spark_env/bin/activate
set -eu

echo "============================================"
echo " Step 1: Build master CSV"
echo " Started: $(date)"
echo "============================================"
"$PYTHON" -u "$SCRIPT_DIR/build_experiment_master_list.py" \
    --root "$DATA_ROOT" \
    --output "$MASTER" \
    "$@"
echo "Done: $(date)"

echo ""
echo "============================================"
echo " Step 2: Trajectory PNGs (all experiments)"
echo " Started: $(date)"
echo "============================================"
"$PYTHON" -u "$SCRIPT_DIR/visualize_experiment.py" \
    --master "$MASTER" \
    --output-root "$VIZ_ROOT" \
    --all \
    --skip-video
echo "Done: $(date)"

echo ""
echo "============================================"
echo " Step 3: RGB + depth videos (all experiments)"
echo " Started: $(date)"
echo "============================================"
"$PYTHON" -u "$SCRIPT_DIR/visualize_experiment.py" \
    --master "$MASTER" \
    --output-root "$VIZ_ROOT" \
    --all \
    --skip-trajectory
echo "Done: $(date)"

echo ""
echo "============================================"
echo " All done: $(date)"
echo " CSV:     $MASTER"
echo " Visuals: $VIZ_ROOT"
echo "============================================"
