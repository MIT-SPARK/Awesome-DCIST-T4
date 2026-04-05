#!/bin/bash
set -e

# Source ROS 2 and workspace
source /opt/ros/${ROS_DISTRO:-jazzy}/setup.bash
if [ -f /root/dcist_ws/install/setup.bash ]; then
    source /root/dcist_ws/install/setup.bash
fi

# Export standard environment
export ADT4_WS=/root/dcist_ws
export ADT4_ENV=/opt/venvs

exec "$@"
