ARG PARENT_IMAGE=nvidia/cuda:11.7.1-cudnn8-devel-ubuntu20.04
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
    python3-pip \
    python3-dev \
    software-properties-common && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/*

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

# install additional dependencies for se3dif package
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
    libsuitesparse-dev \
    build-essential && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/* 

RUN python3 -m pip install --upgrade pip

# here we install all the dependecies defined in the requirements.txt file of thr grasp_diffusion package
# we need to use different versions of some packages than used in the original repo in order to ensure compatibility
# for this reason we install the packages manually here and use the --no-deps flag to avoid installing the original versions in thr next step
# versions that are changed in comparison to the original requirements.txt file are:
# - numpy==1.21.6 -> numpy==1.23.5
RUN python3 -m pip install torch==1.13.1+cu117 torchvision==0.14.1+cu117 torchaudio==0.13.1 --extra-index-url https://download.pytorch.org/whl/cu117
RUN python3 -m pip install \
    numpy==1.23.5 \ 
    theseus-ai==0.1.3 \
    tqdm==4.64.1 \
    configargparse==1.5.3 \
    h5py==3.7.0 \
    pyglet==1.5.27 \
    scikit-learn==1.0.2 \
    trimesh==3.14 \
    matplotlib==3.5.3 \
    torchvision==0.14.1 \
    tensorboard==2.10.0 \
    natsort==8.2.0 \
    pytorch3d

# installation of the package itself
# we need to provide the -e flag because otherwise some submodukes are not installed
# therfore we first need to clone the repository into the docker container and can not use pip install directly or the setup.py file
RUN git clone -b master https://github.com/TheCamusean/grasp_diffusion /home/grasp_diffusion && \
    python3 -m pip install --no-deps -e /home/grasp_diffusion 

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
    pip3 install -e .; \
    fi

ENV GRASP_BENCHMARK_SE3DIF true

CMD ["bash", "-c", "source /home/catkin_ws/devel/setup.bash && roslaunch grasping_benchmarks_ros grasp_planning_benchmark.launch"]