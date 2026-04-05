# dcist-deploy: Multi-stage deployment image for robots
# Stage 1 builds the workspace, Stage 2 creates a slim runtime image.
#
# Usage:
#   docker build -f docker/deploy.Dockerfile -t dcist-deploy:latest \
#     --build-arg PARALLEL_JOBS=2 ..

# ============================================================
# Stage 1: Builder
# ============================================================
ARG BASE_IMAGE=dcist-dev:latest
FROM ${BASE_IMAGE} AS builder

ARG PARALLEL_JOBS=4

COPY . /root/dcist_ws/src/awesome_dcist_t4/

WORKDIR /root/dcist_ws

# Clone additional repos from packages.yaml if not present
RUN if [ ! -d src/pose_graph_tools ]; then \
      apt-get update && apt-get install -y python3-vcstool && \
      vcs import src < src/awesome_dcist_t4/install/packages.yaml || true; \
    fi

# Build the workspace
RUN source /opt/ros/jazzy/setup.bash && \
    MAKEFLAGS="-j${PARALLEL_JOBS}" colcon build \
      --continue-on-error \
      --parallel-workers ${PARALLEL_JOBS} \
      --cmake-args -DCMAKE_BUILD_TYPE=Release

# ============================================================
# Stage 2: Runtime (slim)
# ============================================================
FROM nvidia/cuda:12.8.0-runtime-ubuntu24.04

ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=jazzy

# ---- ROS 2 Jazzy (runtime only, no desktop/dev) ----
RUN apt-get update && \
    apt-get install -y curl gnupg2 lsb-release wget && \
    curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
      -o /etc/apt/trusted.gpg.d/ros.gpg && \
    echo "deb http://packages.ros.org/ros2/ubuntu noble main" \
      > /etc/apt/sources.list.d/ros2.list && \
    apt-get update && \
    apt-get install -y \
      ros-${ROS_DISTRO}-ros-base \
      ros-${ROS_DISTRO}-rmw-zenoh-cpp \
      python3-rosdep \
      python3-colcon-common-extensions \
      tmux && \
    rm -rf /var/lib/apt/lists/*

RUN rosdep init && rosdep update

# ---- Runtime rosdep dependencies ----
COPY --from=builder /root/dcist_ws/src /tmp/src
RUN apt-get update && \
    rosdep install --from-paths /tmp/src --ignore-src -r -y \
      --rosdistro=${ROS_DISTRO} --dependency-types=exec || true && \
    rm -rf /tmp/src /var/lib/apt/lists/*

# ---- Copy built workspace ----
COPY --from=builder /root/dcist_ws/install /root/dcist_ws/install
COPY --from=builder /root/dcist_ws/src/awesome_dcist_t4/dcist_launch_system \
                    /root/dcist_ws/src/awesome_dcist_t4/dcist_launch_system

# ---- Copy Python venvs ----
COPY --from=builder /opt/venvs /opt/venvs

# ---- Copy model weights ----
COPY --from=builder /root/dcist_ws/weights /root/dcist_ws/weights

# ---- Copy fast-downward ----
COPY --from=builder /opt/fast_downward /opt/fast_downward
RUN ln -sf /opt/fast_downward/fast-downward.py /usr/local/bin/fast-downward

# ---- Entrypoint ----
COPY docker/scripts/entrypoint-deploy.sh /entrypoint-deploy.sh
RUN chmod +x /entrypoint-deploy.sh

ENV ADT4_WS=/root/dcist_ws
ENV ADT4_ENV=/opt/venvs
ENV RMW_IMPLEMENTATION=rmw_zenoh_cpp

WORKDIR /root/dcist_ws

ENTRYPOINT ["/entrypoint-deploy.sh"]
CMD ["bash"]
