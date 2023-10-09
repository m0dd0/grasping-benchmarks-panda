#!/bin/bash
set -e # exit on first error

echo "running ros_entrypoint.sh"

# setup ros environment
source "/opt/ros/$ROS_DISTRO/setup.bash" --
source $CATKIN_WS_DIR/devel/setup.bash

# execute any command entered after the docker run command
echo "execute command $@"
exec "$@"