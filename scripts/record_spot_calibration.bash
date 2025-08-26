#!/bin/bash
ros2 bag record \
    /tf \
    /tf_static \
    /${ADT4_ROBOT_NAME}/${ADT4_ROBOT_NAME}_zed/rgb/image_rect_color/compressed \
    /${ADT4_ROBOT_NAME}/${ADT4_ROBOT_NAME}_zed/rgb/camera_info \
    /${ADT4_ROBOT_NAME}/frontleft/color/image_raw/compressed \
    /${ADT4_ROBOT_NAME}/frontleft/color/camera_info \
    /${ADT4_ROBOT_NAME}/frontright/color/image_raw/compressed \
    /${ADT4_ROBOT_NAME}/frontright/color/camera_info \
    $@
