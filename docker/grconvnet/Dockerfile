FROM ubuntu:focal

ARG CATKIN_WS_DIR=/home/catkin_ws
ARG BENCHMARK_REPO=https://github.com/m0dd0/grasping-benchmarks-panda.git
ARG BENCHMARK_BRANCH=develop
ARG GRCONVNET_REPO=https://github.com/m0dd0/GrConvNetBenchmark.git
ARG GRCONVNET_BRANCH=develop

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
    echo "source ${CATKIN_WS_DIR}/devel/setup.bash" >> /etc/bash.bashrc && \
    echo "echo 'ROS environment initialized'" >> /etc/bash.bashrc

# clone and build grasping-benchmarks-panda
RUN mkdir -p ${CATKIN_WS_DIR}/src && \
    cd ${CATKIN_WS_DIR}/src && \
    git clone -b ${BENCHMARK_BRANCH} ${BENCHMARK_REPO} && \
    cd ${CATKIN_WS_DIR} && \
    rosdep update && \
    rosdep install --from-paths src --ignore-src -y -r && \
    catkin config  --extend /opt/ros/noetic && \
    catkin build && \
    cd ${CATKIN_WS_DIR}/src/grasping-benchmarks-panda && \
    pip3 install -e .[grconvnet]

# install grconvnet
RUN cd /home && \
    git clone -b ${GRCONVNET_BRANCH} ${GRCONVNET_REPO} && \
    cd GrConvNetBenchmark && \
    pip3 install -e .

ENV GRASP_BENCHMARK_GRCONVNET true

# setup entrypoint
COPY ./entrypoint.sh /

ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]