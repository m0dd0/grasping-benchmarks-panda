ARG PARENT_IMAGE=nvidia/cuda:11.8.0-cudnn8-devel-ubuntu20.04
FROM ${PARENT_IMAGE}  

ARG CATKIN_WS_DIR=/home/catkin_ws
ARG BENCHMARK_REPO=https://github.com/m0dd0/grasping-benchmarks-panda.git
ARG BENCHMARK_BRANCH=contact_new
ARG CONTACT_GRASPNET_REPO=https://github.com/NVlabs/contact_graspnet
ARG CONTACT_GRASPNET_BRANCH=main

ENV LANG C.UTF-8
ENV LC_ALL C.UTF-8

RUN echo 'Etc/UTC' > /etc/timezone && \
    ln -s /usr/share/zoneinfo/Etc/UTC /etc/localtime && \
    apt-get update && \
    apt-get install -q -y --no-install-recommends tzdata && \
    rm -rf /var/lib/apt/lists/*

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

# create and activate conda environment with contact_graspnet dependencies
RUN cd /home && \
    git clone -b ${CONTACT_GRASPNET_BRANCH} ${CONTACT_GRASPNET_REPO} && \
    conda env create -f ./contact_graspnet/contact_graspnet_env.yml

# RUN cd /home/contact_graspnet && \
#     conda activate contact_graspnet_env && \
#     bash compile_pointnet_tfops.sh

# copy the models manually as automated download is not reliable
COPY ./contact_graspnet_models/ /home/contact_graspnet/checkpoints
COPY ./test_data/ /home/contact_graspnet/test_data
# download contact graspnet models
# note: google has very strict rate limits for automated downloads, so this is not a reliable way to download the models ...
# RUN pip install gdown && \
#     gdown --folder --speed 2MB https://drive.google.com/drive/folders/1tBHKf60K8DLM5arm-Chyf7jxkzOr5zGl && \
#     gdown --folder --speed 2MB -O ./contact_graspnet/ https://drive.google.com/drive/folders/1v0_QMTUIEOcu09Int5V6N2Nuq7UCtuAA && \
#     mv ./contact_graspnet_models/* ./contact_graspnet/checkpoints/ && \
#     rm -rf ./contact_graspnet_models

# RUN git clone -b ${BENCHMARK_BRANCH} ${BENCHMARK_REPO} && \
#     cd grasping-benchmarks-panda && \
#     pip3 install -e .[contact_graspnet]

# ENV GRASP_BENCHMARK_CONTACT true

CMD ["bash"]
# CMD ["roslaunch grasping_benchmarks_ros grasp_planning_benchmark.launch"]
