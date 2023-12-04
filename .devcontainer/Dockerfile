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
    wget \
    build-essential \
    ffmpeg \
    libsm6 \
    libxext6 \
    freeglut3-dev \
    python3-pip \
    ca-certificates \
    bzip2 \
    libglib2.0-0 \
    libxrender1 \
    mercurial \
    openssh-client \
    procps && \ 
    apt-get clean && \
    rm -rf /var/lib/apt/lists/* && \
    pip3 install --upgrade pip

# install conda, adapted from https://github.com/ContinuumIO/docker-images/blob/main/miniconda3/debian/Dockerfile
ENV PATH /opt/conda/bin:$PATH
ARG CONDA_VERSION=py311_23.10.0-1
RUN set -x && \
    UNAME_M="$(uname -m)" && \
    if [ "${UNAME_M}" = "x86_64" ]; then \
    MINICONDA_URL="https://repo.anaconda.com/miniconda/Miniconda3-${CONDA_VERSION}-Linux-x86_64.sh"; \
    SHA256SUM="d0643508fa49105552c94a523529f4474f91730d3e0d1f168f1700c43ae67595"; \
    elif [ "${UNAME_M}" = "s390x" ]; then \
    MINICONDA_URL="https://repo.anaconda.com/miniconda/Miniconda3-${CONDA_VERSION}-Linux-s390x.sh"; \
    SHA256SUM="ae212385c9d7f7473da7401d3f5f6cbbbc79a1fce730aa48531947e9c07e0808"; \
    elif [ "${UNAME_M}" = "aarch64" ]; then \
    MINICONDA_URL="https://repo.anaconda.com/miniconda/Miniconda3-${CONDA_VERSION}-Linux-aarch64.sh"; \
    SHA256SUM="a60e70ad7e8ac5bb44ad876b5782d7cdc66e10e1f45291b29f4f8d37cc4aa2c8"; \
    elif [ "${UNAME_M}" = "ppc64le" ]; then \
    MINICONDA_URL="https://repo.anaconda.com/miniconda/Miniconda3-${CONDA_VERSION}-Linux-ppc64le.sh"; \
    SHA256SUM="1a2eda0a9a52a4bd058abbe9de5bb2bc751fcd7904c4755deffdf938d6f4436e"; \
    fi && \
    wget "${MINICONDA_URL}" -O miniconda.sh -q && \
    echo "${SHA256SUM} miniconda.sh" > shasum && \
    if [ "${CONDA_VERSION}" != "latest" ]; then sha256sum --check --status shasum; fi && \
    mkdir -p /opt && \
    bash miniconda.sh -b -p /opt/conda && \
    rm miniconda.sh shasum && \
    ln -s /opt/conda/etc/profile.d/conda.sh /etc/profile.d/conda.sh && \
    echo ". /opt/conda/etc/profile.d/conda.sh" >> ~/.bashrc && \
    echo "conda activate base" >> ~/.bashrc && \
    find /opt/conda/ -follow -type f -name '*.a' -delete && \
    find /opt/conda/ -follow -type f -name '*.js.map' -delete && \
    /opt/conda/bin/conda clean -afy

# conda requires login shell
SHELL ["/bin/bash", "--login", "-c"] 
RUN conda init bash

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
RUN git clone -b develop https://github.com/m0dd0/ContactGraspnetBenchmark.git /home/ContactGraspnetBenchmark
    # cd /home/ContactGraspnetBenchmark && \
    # pip3 install -e .

ENV GRASP_BENCHMARK_CONTACT true

CMD ["bash"]