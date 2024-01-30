# cd /home/catkin_ws 
# rosdep update 
# rosdep install --from-paths src --ignore-src -y -r 
# catkin config --extend /opt/ros/noetic 
# catkin build 
# python3 -m pip install -e /home/catkin_ws/src/grasping-benchmark-panda[se3dif]