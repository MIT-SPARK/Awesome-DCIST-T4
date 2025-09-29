#!/bin/bash
#https://drive.google.com/file/d/1XRcsyLSvqqhqNIaOI_vmqpUpmBT6gk9-/view?usp=drive_link
set -e

# Path for the first-run flag
FIRST_RUN_FLAG="/root/.first_run_complete"
MODEL_DIR="/root/semantic_inference"

# --- CORRECTED MODEL INFORMATION ---
# The correct Google Drive ID for the project's intended model (ade20k-hrnetv2-c1.engine)
MODEL_FILE_ID="1XRcsyLSvqqhqNIaOI_vmqpUpmBT6gk9-"
# The exact filename the system is configured to look for by default
MODEL_FILENAME="ade20k-efficientvit_seg_l2.onnx"
MODEL_PATH="${MODEL_DIR}/${MODEL_FILENAME}"

# First-run setup: Download the intended model and save it with the expected filename
if [ ! -f "$FIRST_RUN_FLAG" ]; then
    echo "--- First run detected: Setting up semantic inference model ---"
    
    if ! command -v gdown &> /dev/null; then
        echo "FATAL: gdown command not found. Please check the Dockerfile."
        exit 1
    fi
    
    echo "Downloading project model and saving as '${MODEL_FILENAME}'..."
    mkdir -p "$MODEL_DIR"
    
    # Download the correct file but save it with the name the launch file expects
    if gdown --id "$MODEL_FILE_ID" -O "$MODEL_PATH"; then
        echo "Model downloaded successfully to $MODEL_PATH"
        # Create the flag file to indicate successful completion
        touch "$FIRST_RUN_FLAG"
    else
        echo "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
        echo "!!! WARNING: Failed to download semantic inference model from Google Drive.!!!"
        echo "!!! The container will start, but the semantic inference node will fail.   !!!"
        echo "!!! This may be due to a network issue or Google Drive quotas.             !!!"
        echo "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
        # Do not create the flag file, so it will retry on the next run
    fi
    echo "--- First run setup complete ---"
fi


# Source ROS 2 and the workspace
# Note: .zshrc sources the environment, but we do it here for non-interactive shells.
source /opt/ros/jazzy/setup.bash
if [ -f /dcist_ws/install/setup.bash ]; then
    source /dcist_ws/install/setup.bash
fi

# Execute the command passed to the container (e.g., "zsh")
exec "$@"