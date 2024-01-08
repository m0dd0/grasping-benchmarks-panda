#!/bin/bash
set -e # exit on first error

# check for new commits in the grasping benchmark repo and rebuild the catkin workspace if necessary
echo "### Checking for new commits in the grasping-benchmarks-panda repo."
cd $CATKIN_WS_DIR/src/grasping-benchmarks-panda
current_commit=$(git rev-parse HEAD)
git pull
new_commit=$(git rev-parse HEAD)
if [ "$current_commit" != "$new_commit" ];
then
    echo "### Rebuilding catkin workspace because new commits were pulled for grasping-benchmarks-panda ($current_commit -> $new_commit). Consider rebuilding the docker image to avoid this step in the future."
    cd $CATKIN_WS_DIR
    catkin clean -y
    catkin build
    cd $CATKIN_WS_DIR/src/grasping-benchmarks-panda
    pip3 install -e .[grconvnet]
fi

# check for new commits on the grconvnet repo and reinstall the python package if necessary
echo "### Checking for new commits in the grconvnet repo."
cd /home/GrConvNetBenchmark
current_commit=$(git rev-parse HEAD)
git pull
new_commit=$(git rev-parse HEAD)
if [ "$current_commit" != "$new_commit" ];
then
    echo "### Reinstalling grconvnet because new commits were pulled ($current_commit -> $new_commit). Consider rebuilding the docker image to avoid this step in the future."
    pip3 install -e .
fi

# execute any command entered after the docker run command
echo "execute command $@"
exec "$@"