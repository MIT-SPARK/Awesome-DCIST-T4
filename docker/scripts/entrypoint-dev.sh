#!/bin/bash
set -e

# Source ROS 2
source /opt/ros/${ROS_DISTRO:-jazzy}/setup.bash

# ---- Populate venvs if volume is empty ----
# Docker named volumes auto-initialize from image contents on first creation.
# This fallback handles the case where someone explicitly deletes the volume.
if [ ! -f /opt/venvs/roman/bin/activate ]; then
    echo "[entrypoint] Venvs volume is empty. Creating roman from scratch..."
    if [ -f /root/dcist_ws/src/awesome_dcist_t4/install/python_setup.bash ]; then
        cd /root/dcist_ws/src/awesome_dcist_t4/install
        ADT4_ENV=/opt/venvs ADT4_WS=/root/dcist_ws \
            bash python_setup.bash --no-spark || echo "[entrypoint] WARNING: roman venv setup failed"
    fi
fi

if [ ! -f /opt/venvs/spark_env/bin/activate ]; then
    echo "[entrypoint] Venvs volume is empty. Creating spark_env from scratch..."
    if [ -f /root/dcist_ws/src/awesome_dcist_t4/install/python_setup.bash ]; then
        cd /root/dcist_ws/src/awesome_dcist_t4/install
        ADT4_ENV=/opt/venvs ADT4_WS=/root/dcist_ws \
            bash python_setup.bash --no-roman || echo "[entrypoint] WARNING: spark_env setup failed"
    fi
fi

# ---- Seed model weights from image cache ----
WEIGHTS_DIR=/root/dcist_ws/weights
if [ -d /tmp/weights ] && [ ! -f "$WEIGHTS_DIR/yolov8s-world.pt" ]; then
    echo "[entrypoint] Copying pre-downloaded model weights..."
    mkdir -p "$WEIGHTS_DIR"
    cp -n /tmp/weights/* "$WEIGHTS_DIR/" 2>/dev/null || true
fi

# ---- Source workspace overlay if built ----
if [ -f /root/dcist_ws/install/setup.bash ]; then
    source /root/dcist_ws/install/setup.bash
fi

exec "$@"
