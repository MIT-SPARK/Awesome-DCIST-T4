#!/bin/bash

if [ -z "$1" ]; then
    echo "Usage: $0 <folder_name>"
    exit 1
fi

FOLDER_NAME="$1"
SAVE_DIR="/home/swarm/data/spot_manipulation_data/$FOLDER_NAME"
CSV_FILE="$SAVE_DIR/trial_summary.csv"

# Create the target folder
mkdir -p "$SAVE_DIR"

# Function to handle CTRL-C
function on_interrupt {
    echo ""
    echo "Bagging interrupted."

    read -rp "Would you like to save the trial? (Type 'no' to discard): " save_response
    save_response_lower=$(echo "$save_response" | tr '[:upper:]' '[:lower:]')
    if [[ "$save_response_lower" == "n" || "$save_response_lower" == "no" ]]; then
        echo "Trial will not be saved. Deleting folder..."
        rm -rf "$SAVE_DIR"
        echo "Folder deleted. Exiting."
        exit 0
    fi

    # Collect metadata
    read -rp "How many times did the robot reject the grasp (fail to find a solution)? " reject
    read -rp "How many times did the robot detect the object but miss? " miss
    read -rp "How many times did the robot succeed to pick up the object? " success
    read -rp "Please add any additional information about the trial: " summary

    # Save to CSV
    echo "Reject, Miss, Success, Summary" > "$CSV_FILE"
    echo "$reject, $miss, $success, \"$summary\"" >> "$CSV_FILE"

    echo "Summary saved to $CSV_FILE"
    exit 0
}

# Trap CTRL-C
trap on_interrupt SIGINT

# Start bagging
ros2 bag record -o "$SAVE_DIR/bag" \
     /tf \
     /tf_static \
     /${ADT4_ROBOT_NAME}/${ADT4_ROBOT_NAME}_zed/depth/camera_info \
     /${ADT4_ROBOT_NAME}/${ADT4_ROBOT_NAME}_zed/depth/depth_registered/compressedDepth \
     /${ADT4_ROBOT_NAME}/${ADT4_ROBOT_NAME}_zed/rgb/image_rect_color/compressed \
     /${ADT4_ROBOT_NAME}/${ADT4_ROBOT_NAME}_zed/rgb/camera_info \
     /${ADT4_ROBOT_NAME}/frontleft/color/camera_info \
     /${ADT4_ROBOT_NAME}/frontleft/color/image_raw/compressed \
     /${ADT4_ROBOT_NAME}/frontleft/depth/camera_info \
     /${ADT4_ROBOT_NAME}/frontleft/depth/image_rect \
     /${ADT4_ROBOT_NAME}/frontright/color/camera_info \
     /${ADT4_ROBOT_NAME}/frontright/color/image_raw/compressed \
     /${ADT4_ROBOT_NAME}/frontright/depth/camera_info \
     /${ADT4_ROBOT_NAME}/frontright/depth/image_rect \
     /${ADT4_ROBOT_NAME}/joint_states \
     /${ADT4_ROBOT_NAME}/hand/color/camera_info \
     /${ADT4_ROBOT_NAME}/hand/color/image_raw/compressed \
     /${ADT4_ROBOT_NAME}/hand/depth/camera_info \
     /${ADT4_ROBOT_NAME}/hand/depth/image_rect \
     $@
