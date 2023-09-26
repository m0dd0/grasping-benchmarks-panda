#!/bin/bash
set -e # exit on first error

# setup ros environment
source "/opt/ros/$ROS_DISTRO/setup.bash" --

# execute any command entered after the docker run command
echo "execute command $@"
exec "$@"