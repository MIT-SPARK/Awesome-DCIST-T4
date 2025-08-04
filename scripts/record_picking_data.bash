#!/bin/bash

if [ -z "$1" ]; then
    echo "Usage: $0 <folder_name>"
    exit 1
fi

FOLDER_NAME="$1"
SAVE_DIR="/home/swarm/data/spot_manipulation_data/$FOLDER_NAME"

# Create the target folder
mkdir -p "$SAVE_DIR"

# When the user CTRL-C's, it prompts the user to write metadata about the trial. 
function on_interrupt {
    echo ""
    echo "Bagging interrupted. Please enter a summary of the trial. Start your string with 0 for unsuccessful and 1 for successful:"
    read -rp "Trial Summary: " summary
    echo "$summary" > "$SAVE_DIR/trial_summary.txt"
    echo "Summary saved to $SAVE_DIR/trial_summary.txt"
    exit 0
}

trap on_interrupt SIGINT


ros2 bag record -o "$SAVE_DIR/bag" \
     /tf \
     /tf_static \
     /${ADT4_ROBOT_NAME}/${ADT4_ROBOT_NAME}_zed/depth/camera_info \
     /${ADT4_ROBOT_NAME}/${ADT4_ROBOT_NAME}_zed/depth/depth_registered \
     /${ADT4_ROBOT_NAME}/${ADT4_ROBOT_NAME}_zed/rgb/image_rect_color \
     /${ADT4_ROBOT_NAME}/${ADT4_ROBOT_NAME}_zed/rgb/camera_info \
     /${ADT4_ROBOT_NAME}/joint_states \
     $@

