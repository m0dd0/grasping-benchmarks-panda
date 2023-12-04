ARG PARENT_IMAGE=nvidia/cuda:11.8.0-cudnn8-devel-ubuntu20.04
FROM ${PARENT_IMAGE}  

# setup environment (variables)
ENV LANG C.UTF-8
ENV LC_ALL C.UTF-8
ENV ROS_DISTRO noetic
ENV CATKIN_WS_DIR ${CATKIN_WS_DIR}

# setup timezone
RUN echo 'Etc/UTC' > /etc/timezone && \
    ln -s /usr/share/zoneinfo/Etc/UTC /etc/localtime && \
    apt-get update && \
    apt-get install -q -y --no-install-recommends tzdata && \
    rm -rf /var/lib/apt/lists/*

# install basic packages
RUN apt-get update && \ 
    apt-get install -q -y --no-install-recommends \
    dirmngr \ 
    gnupg2 \
    git \
    nano \
    curl \
    make \
    openssl \
    build-essential \
    python3-pip \
    && rm -rf /var/lib/apt/lists/*

# setup sources.list and keys for ros packages
RUN echo "deb http://packages.ros.org/ros/ubuntu focal main" > /etc/apt/sources.list.d/ros1-latest.list && \
    apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-keys C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654

# install ros packages
RUN apt-get update && \ 
    apt-get install -y --no-install-recommends \
    ros-noetic-ros-core=1.5.0-1* \
    python3-catkin-tools && \
    rm -rf /var/lib/apt/lists/* && \
    pip install -U rosdep && \
    rosdep init && \
    echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /etc/bash.bashrc && \
    echo "source /home/catkin_ws/devel/setup.bash" >> /etc/bash.bashrc && \
    echo "echo 'ROS environment initialized'" >> /etc/bash.bashrc

# clone and build grasping-benchmarks-panda
RUN mkdir -p /home/catkin_ws/src
# cd ${CATKIN_WS_DIR}/src && \
# git clone -b ${BENCHMARK_BRANCH} ${BENCHMARK_REPO} && \
# cd ${CATKIN_WS_DIR} && \
# rosdep update && \
# rosdep install --from-paths src --ignore-src -y -r && \
# catkin config  --extend /opt/ros/noetic && \
# catkin build && \
# cd ${CATKIN_WS_DIR}/src/grasping-benchmarks-panda && \
# pip3 install -e .

# install packaged and partly refactored version of contact_graspnet
RUN git clone -b develop https://github.com/m0dd0/ContactGraspnetBenchmark.git /home/ContactGraspnetBenchmark && \
    cd /home/ContactGraspnetBenchmark && \
    pip3 install -e .

ENV GRASP_BENCHMARK_CONTACT true

CMD ["bash"]