FROM tensorflow/tensorflow:2.12.0-gpu

ARG CATKIN_WS_DIR=/home/catkin_ws
ARG BENCHMARK_REPO=https://github.com/m0dd0/grasping-benchmarks-panda.git
ARG BENCHMARK_BRANCH=contact_graspnet
ARG CONTACT_GRASPNET_REPO=https://github.com/m0dd0/ContactGraspnetBenchmark.git
ARG CONTACT_GRASPNET_BRANCH=develop

# setup environment
ENV LANG C.UTF-8
ENV LC_ALL C.UTF-8
ENV ROS_DISTRO noetic
ENV CATKIN_WS_DIR ${CATKIN_WS_DIR}

# setup timezone
RUN if [! -e /etc/timezone ]; then \
    echo 'Etc/UTC' > /etc/timezone; \
    ln -s /usr/share/zoneinfo/Etc/UTC /etc/localtime; \
    fi && \
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
    wget \
    build-essential \
    python3-pip && \
    pip3 install --upgrade pip && \
    rm -rf /var/lib/apt/lists/*

# install ros
RUN echo "deb http://packages.ros.org/ros/ubuntu focal main" > /etc/apt/sources.list.d/ros1-latest.list && \
    apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-keys C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654 && \
    apt-get update && \ 
    apt-get install -y --no-install-recommends \
    ros-noetic-ros-core=1.5.0-1* \
    python3-catkin-tools && \
    rm -rf /var/lib/apt/lists/* && \
    pip install -U rosdep && \
    rosdep init && \
    echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /etc/bash.bashrc && \
    echo "source ${CATKIN_WS_DIR}/devel/setup.bash" >> /etc/bash.bashrc && \
    echo "echo 'ROS environment initialized'" >> /etc/bash.bashrc

# install conda and create conda environment
# this is needed since the tensorflow docker image somehow prevents local pip installs, 
# however installing into a conda environment works
ENV PATH="/root/miniconda3/bin:${PATH}"
ARG PATH="/root/miniconda3/bin:${PATH}"
RUN wget https://repo.anaconda.com/miniconda/Miniconda3-latest-Linux-x86_64.sh && \ 
    mkdir /root/.conda && \
    bash Miniconda3-latest-Linux-x86_64.sh -b && \ 
    rm -f Miniconda3-latest-Linux-x86_64.sh && \
    conda create -n contact_graspnet python=3.10 && \
    conda init bash
SHELL ["conda", "run", "-n", "contact_graspnet", "/bin/bash", "-c"]

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
    pip3 install -e .[contact_graspnet]

# install contact_graspnet
RUN cd /home && \
    git clone --recursive -b ${CONTACT_GRASPNET_BRANCH} ${CONTACT_GRASPNET_REPO} && \
    cd ContactGraspnetBenchmark && \
    pip3 install -e .

ENV GRASP_BENCHMARK_CONTACT true

# setup entrypoint
COPY ./entrypoint.sh /

ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
# CMD ["roslaunch grasping_benchmarks_ros grasp_planning_benchmark.launch"]
