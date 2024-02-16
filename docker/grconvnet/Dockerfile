ARG PARENT_IMAGE=ubuntu:focal
FROM ${PARENT_IMAGE}

ARG IS_DEVCONTAINER_BUILD=false
ARG BENCHMARK_REPO=https://github.com/m0dd0/grasping-benchmarks-panda.git
ARG BENCHMARK_BRANCH=develop

# setup environment (variables)
ENV LANG C.UTF-8
ENV LC_ALL C.UTF-8
ENV ROS_DISTRO noetic

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
    ros-noetic-desktop \
    python3-catkin-tools && \
    rm -rf /var/lib/apt/lists/* && \
    pip install -U rosdep && \
    rosdep init && \
    echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /etc/bash.bashrc && \
    echo "source /home/catkin_ws/devel/setup.bash" >> /etc/bash.bashrc && \
    echo "echo 'ROS environment initialized'" >> /etc/bash.bashrc

# install grconvnet
# TODO check if this can be integrated in the setup.py of grasping-benchmarks-panda
RUN cd /home && \
    git clone -b develop https://github.com/m0dd0/GrConvNetBenchmark.git && \
    cd GrConvNetBenchmark && \
    pip3 install -e .

# clone and build grasping-benchmarks-panda
# this is not executed when building the devcontainer as the workspace is mounted from the host and installed in postCreateCommand
RUN if [ "${IS_DEVCONTAINER_BUILD}" = "false" ]; then \
    mkdir -p /home/catkin_ws/src && \
    cd /home/catkin_ws/src && \
    git clone -b ${BENCHMARK_BRANCH} ${BENCHMARK_REPO} && \
    cd /home/catkin_ws && \
    rosdep update && \
    rosdep install --from-paths src --ignore-src -y -r && \
    catkin config  --extend /opt/ros/noetic && \
    catkin build && \
    cd /home/catkin_ws/src/grasping-benchmarks-panda && \
    pip3 install -e .[grconvnet]; \
    fi

ENV GRASP_BENCHMARK_GRCONVNET true

CMD ["bash", "-c", "source /home/catkin_ws/devel/setup.bash && roslaunch grasping_benchmarks_ros grasp_planning_benchmark.launch"]