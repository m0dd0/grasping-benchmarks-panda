FROM nvidia/cuda:11.8.0-base-ubuntu20.04

# setup timezone
RUN if [! -e /etc/timezone ]; then \
    echo 'Etc/UTC' > /etc/timezone; \
    ln -s /usr/share/zoneinfo/Etc/UTC /etc/localtime; \
    fi && \
    apt-get update && \
    apt-get install -q -y --no-install-recommends tzdata && \
    rm -rf /var/lib/apt/lists/*

# Install some basic utilities
RUN apt-get update && apt-get install -y \
    curl \
    wget \
    libxml2 \
    gcc \
    git \
    python3-pip \
    && rm -rf /var/lib/apt/lists/*

RUN pip3 install tensorflow[and-cuda]

# RUN wget https://developer.download.nvidia.com/compute/cuda/11.8.0/local_installers/cuda_11.8.0_520.61.05_linux.run && \
#     chmod +x cuda_11.8.0_520.61.05_linux.run && \
#     sh cuda_11.8.0_520.61.05_linux.run

# RUN wget https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2004/x86_64/cuda-ubuntu2004.pin && \
#     mv cuda-ubuntu2004.pin /etc/apt/preferences.d/cuda-repository-pin-600 && \
#     wget https://developer.download.nvidia.com/compute/cuda/11.8.0/local_installers/cuda-repo-ubuntu2004-11-8-local_11.8.0-520.61.05-1_amd64.deb && \
#     dpkg -i cuda-repo-ubuntu2004-11-8-local_11.8.0-520.61.05-1_amd64.deb && \
#     cp /var/cuda-repo-ubuntu2004-11-8-local/cuda-*-keyring.gpg /usr/share/keyrings/ && \
#     apt-get update && \
#     apt-get -y install cuda