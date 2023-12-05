cd /home/catkin_ws 
rosdep update 
rosdep install --from-paths src --ignore-src -y -r 
catkin config --extend /opt/ros/noetic 
catkin build 
pip install -e /home/catkin_ws/src/grasping-benchmark-panda
pip install -r /home/catkin_ws/src/grasping-benchmark-panda/.devcontainer/requirements.txt