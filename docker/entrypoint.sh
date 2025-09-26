#!/bin/bash
set -e

# Path for the first-run flag
FIRST_RUN_FLAG="/root/.first_run_complete"
MODEL_DIR="/root/.semantic_inference"
MODEL_FILE_ID="1XRcsyLSvqqhqNIaOI_vmqpUpmBT6gk9-"
MODEL_PATH="${MODEL_DIR}/ade20k-hrnetv2-c1.engine"

# First-run setup: Download the semantic inference model
if [ ! -f "$FIRST_RUN_FLAG" ]; then
    echo "--- First run detected: Setting up semantic inference model ---"
    
    # Check if gdown is installed
    if ! command -v gdown &> /dev/null; then
        echo "gdown not found. Installing..."
        pip install gdown
    fi
    
    echo "Downloading model weights from Google Drive..."
    mkdir -p "$MODEL_DIR"
    gdown --id "$MODEL_FILE_ID" -O "$MODEL_PATH"
    
    if [ $? -eq 0 ]; then
        echo "Model downloaded successfully to $MODEL_PATH"
        # Create the flag file to indicate successful completion
        touch "$FIRST_RUN_FLAG"
    else
        echo "!!! Failed to download model. Please check the network or link. !!!"
        # Do not create the flag file, so it will retry on the next run
    fi
    echo "--- First run setup complete ---"
fi


# Source ROS 2 and the workspace (if it exists)
source /opt/ros/jazzy/setup.bash
if [ -f /dcist_ws/install/setup.bash ]; then
    echo "Sourcing DCIST workspace..."
    source /dcist_ws/install/setup.bash
fi

# Execute the command passed to the container (e.g., "bash")
exec "$@"