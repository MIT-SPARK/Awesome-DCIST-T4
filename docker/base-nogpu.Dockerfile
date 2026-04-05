# dcist-base-nogpu: System dependencies + ROS 2 Jazzy (no CUDA/TensorRT)
# For developers without NVIDIA GPUs.
# Usage: docker build -f docker/base-nogpu.Dockerfile -t dcist-base:latest .

FROM osrf/ros:jazzy-desktop

ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=jazzy

# ---- Additional ROS packages ----
RUN apt-get update && \
    apt-get install -y \
      python3-rosdep \
      python3-colcon-common-extensions \
      ros-${ROS_DISTRO}-depth-image-proc \
      ros-${ROS_DISTRO}-rqt-tf-tree && \
    rm -rf /var/lib/apt/lists/*

# rosdep may already be initialized in osrf image; ignore errors
RUN rosdep init 2>/dev/null || true && rosdep update

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

# ---- Zenoh RMW ----
RUN apt-get update && \
    apt-get install -y ros-${ROS_DISTRO}-rmw-zenoh-cpp && \
    rm -rf /var/lib/apt/lists/*
