#!/bin/bash
set -e # exit on first error

# check for new commits in the grasping benchmark repo and rebuild the catkin workspace if necessary
cd $CATKIN_WS_DIR/src/grasping-benchmarks-panda
current_commit=$(git rev-parse HEAD)
git pull
new_commit=$(git rev-parse HEAD)
if [ "$current_commit" != "$new_commit" ];
then
    echo "rebuilding catkin workspace because new commits were pulled for grasping-benchmarks-panda ($current_commit -> $new_commit). Consider rebuilding the docker image to avoid this step in the future."
    cd $CATKIN_WS_DIR
    catkin clean -y
    catkin build
    cd $CATKIN_WS_DIR/src/grasping-benchmarks-panda
    pip3 install -e .[grconvnet]
else
    echo "grasping-benchmarks-panda is up to date ($current_commit)"
fi

# check for new commits on the grconvnet repo and reinstall the python package if necessary
cd /home/GrConvNetBenchmark
current_commit=$(git rev-parse HEAD)
git pull
new_commit=$(git rev-parse HEAD)
if [ "$current_commit" != "$new_commit" ];
then
    echo "reinstalling grconvnet because new commits were pulled ($current_commit -> $new_commit). Consider rebuilding the docker image to avoid this step in the future."
    pip3 install -e .
else
    echo "grconvnet is up to date ($current_commit)"
fi

# execute any command entered after the docker run command
echo "execute command $@"
exec "$@"