#!/bin/bash
ros2 bag record \
    /tf \
    /tf_static \
    /${ADT4_ROBOT_NAME}/${ADT4_ROBOT_NAME}_zed/depth/camera_info \
    /${ADT4_ROBOT_NAME}/${ADT4_ROBOT_NAME}_zed/depth/depth_registered \
    /${ADT4_ROBOT_NAME}/${ADT4_ROBOT_NAME}_zed/rgb/image_rect_color/compressed \
    /${ADT4_ROBOT_NAME}/${ADT4_ROBOT_NAME}_zed/rgb/image_rect_color \
    /${ADT4_ROBOT_NAME}/${ADT4_ROBOT_NAME}_zed/rgb/camera_info \
    /${ADT4_ROBOT_NAME}/joint_states \
    $@
