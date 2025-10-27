#!/usr/bin/env bash
set -e

# Source ROS
[ -f /opt/ros/humble/setup.bash ] && source /opt/ros/humble/setup.bash
[ -f /snoopy/install/setup.bash ] && source /snoopy/install/setup.bash

# Refresh rosdep without prompting for a password
# (Dockerfile should chown /var/cache/rosdep to dev so sudo isn't needed)
rosdep update || true

exec "$@"
