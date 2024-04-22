cd /home/catkin_ws 
rosdep update 
rosdep install --from-paths src --ignore-src -y -r 
catkin config --extend /opt/ros/noetic 
catkin build 
pip install -e /home/catkin_ws/src/grasping-benchmark-panda[dev]
# for some reason the compile_pointnet_script needs to be run manually after the container was build, otherwise no grasps are detected
# the reason for this needs to be investigated