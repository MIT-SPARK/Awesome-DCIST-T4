# dcist-base: System dependencies + ROS 2 Jazzy + CUDA + TensorRT
# This layer changes rarely. Build once and cache.
# Usage: docker build -f docker/base.Dockerfile -t dcist-base:latest .

FROM nvidia/cuda:12.8.0-devel-ubuntu24.04

ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=jazzy

# ---- ROS 2 Jazzy ----
RUN apt-get update && \
    apt-get install -y curl gnupg2 lsb-release software-properties-common && \
    curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
      -o /etc/apt/trusted.gpg.d/ros.gpg && \
    echo "deb http://packages.ros.org/ros2/ubuntu noble main" \
      > /etc/apt/sources.list.d/ros2.list && \
    apt-get update && \
    apt-get install -y \
      ros-${ROS_DISTRO}-desktop \
      python3-rosdep \
      python3-colcon-common-extensions \
      ros-${ROS_DISTRO}-depth-image-proc \
      ros-${ROS_DISTRO}-rqt-tf-tree && \
    rm -rf /var/lib/apt/lists/*

RUN rosdep init && rosdep update

# ---- Build tools and C++ library deps ----
RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-venv \
    python3-virtualenv \
    build-essential \
    cmake \
    usbutils \
    git \
    curl \
    wget \
    tmux \
    vim \
    htop \
    net-tools \
    ros-dev-tools \
    nlohmann-json3-dev \
    libgoogle-glog-dev \
    libcli11-dev \
    libeigen3-dev \
    libzmq3-dev \
    pybind11-dev && \
    rm -rf /var/lib/apt/lists/*

# ---- pipx + rosbags ----
RUN apt-get update && apt-get install -y pipx && \
    pipx install rosbags && \
    rm -rf /var/lib/apt/lists/*

ENV PATH="/root/.local/bin:${PATH}"

# ---- TensorRT 10.8.0.43 for CUDA 12.8 ----
# Must remove conflicting cuda repo lists that ship with the base image
RUN rm -f /etc/apt/sources.list.d/cuda*.list && \
    wget https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2404/x86_64/cuda-keyring_1.1-1_all.deb && \
    dpkg -i cuda-keyring_1.1-1_all.deb && \
    apt-get update && \
    apt-get install -y \
      libnvinfer-dev=10.8.0.43-1+cuda12.8 \
      libnvinfer10=10.8.0.43-1+cuda12.8 \
      libnvinfer-headers-dev=10.8.0.43-1+cuda12.8 \
      libnvinfer-plugin-dev=10.8.0.43-1+cuda12.8 \
      libnvinfer-plugin10=10.8.0.43-1+cuda12.8 \
      libnvinfer-headers-plugin-dev=10.8.0.43-1+cuda12.8 \
      libnvonnxparsers-dev=10.8.0.43-1+cuda12.8 \
      libnvonnxparsers10=10.8.0.43-1+cuda12.8 && \
    rm -rf /var/lib/apt/lists/* && \
    rm cuda-keyring_1.1-1_all.deb

# ---- Zenoh RMW ----
RUN apt-get update && \
    apt-get install -y ros-${ROS_DISTRO}-rmw-zenoh-cpp && \
    rm -rf /var/lib/apt/lists/*
