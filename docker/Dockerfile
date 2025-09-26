# Stage 1: Base image with all dependencies
FROM nvidia/cuda:12.5.1-devel-ubuntu24.04 AS base

# Prevent interactive prompts
ENV DEBIAN_FRONTEND=noninteractive
ENV TZ=Etc/UTC

# Install base utilities and locales, now including tmux, rich, vim, and zsh prerequisites
RUN apt-get update && apt-get install -y \
    locales \
    git \
    wget \
    curl \
    lsb-release \
    gnupg2 \
    sudo \
    zsh \
    tmux \
    python3-rich \
    vim \
    && rm -rf /var/lib/apt/lists/*

# Setup locale
RUN locale-gen en_US en_US.UTF-8 && update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
ENV LANG en_US.UTF-8

# --- Install Oh-My-Zsh and configure ROS sourcing ---
# Install oh-my-zsh non-interactively
RUN sh -c "$(curl -fsSL https://raw.githubusercontent.com/ohmyzsh/ohmyzsh/master/tools/install.sh)" "" --unattended
# Add ROS environment sourcing to the end of the .zshrc file for interactive shells
RUN echo '\n# Source ROS 2 environments' >> /root/.zshrc && \
    echo 'source /opt/ros/jazzy/setup.zsh' >> /root/.zshrc && \
    echo 'if [ -f /dcist_ws/install/setup.zsh ]; then source /dcist_ws/install/setup.zsh; fi' >> /root/.zshrc

# Install ROS 2 Jazzy
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null
RUN apt-get update && apt-get install -y \
    ros-jazzy-ros-base \
    ros-dev-tools \
    ros-jazzy-rmw-zenoh-cpp \
    && rm -rf /var/lib/apt/lists/*

# Initialize rosdep
RUN rosdep init && rosdep update

# Install Awesome DCIST T4 dependencies from README
RUN apt-get update && apt-get install -y \
    pipx \
    python3-virtualenv \
    python3-colcon-common-extensions \
    python3-colcon-clean \
    python3-pip \
    && rm -rf /var/lib/apt/lists/*

# Install python tools with pipx
ENV PATH="/root/.local/bin:${PATH}"
RUN pipx install -f tmuxp
RUN pipx install -f pre-commit
RUN pipx install gdown

# --- Install TensorRT for Semantic Inference ---
RUN apt-get update && apt-get install -y \
    libnvinfer-dev \
    libnvonnxparsers-dev \
    libnvinfer-plugin-dev \
    cuda-nvcc-12-5 \
    && rm -rf /var/lib/apt/lists/*

# Create and set up the colcon workspace
ENV ADT4_WS=/dcist_ws
WORKDIR ${ADT4_WS}
RUN mkdir src

# Set environment variables from the README
ENV ADT4_DLS_PKG=${ADT4_WS}/src/awesome_dcist_t4/dcist_launch_system
ENV ADT4_ENV=/root/environments/dcist
ENV RMW_IMPLEMENTATION=rmw_zenoh_cpp
ENV PATH="${ADT4_WS}/src/awesome_dcist_t4/dcist_launch_system/bin:${PATH}"
ENV SEMANTIC_INFERENCE_MODEL_DIR=/root/semantic_inference

# Copy entrypoint script
COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
# --- Set the default shell to Zsh ---
CMD ["zsh"]

# --------------------------------------------------------------------
# Stage 2: Builder image for deployment
FROM base AS builder

ARG MAKE_JOBS=4
WORKDIR ${ADT4_WS}
SHELL ["/bin/bash", "-c"]

# Instead of cloning, copy the pre-cloned 'adt4_src' from the host context.
COPY adt4_src ./src

# Install ROS dependencies using the copied source code.
RUN apt-get update && \
    source /opt/ros/jazzy/setup.bash && \
    rosdep install \
      --from-paths src \
      --ignore-src \
      -r -y \
      --rosdistro jazzy \
      --skip-keys "opencv rviz gflags_catkin spark_config"

# Set up colcon defaults
RUN echo 'build: {symlink-install: true, cmake-args: [-DCMAKE_BUILD_TYPE=RelWithDebInfo]}' > colcon_defaults.yaml

# Set up Python environments
RUN bash src/awesome_dcist_t4/install/python_setup.bash

# Create symlink for fast-downward
RUN ln -s ${ADT4_WS}/src/fast_downward/fast-downward.py /usr/local/bin/fast-downward

# Build the workspace using the MAKE_JOBS argument to control RAM usage.
RUN source /opt/ros/jazzy/setup.bash && \
    MAKEFLAGS="-j${MAKE_JOBS}" colcon build --continue-on-error --executor sequential

# Perform the manual pip install for spark_dsg as recommended
RUN source /opt/ros/jazzy/setup.bash && \
    source ${ADT4_ENV}/spark_env/bin/activate && \
    pip install ${ADT4_WS}/src/awesome_dcist_t4/spark_dsg

# --------------------------------------------------------------------
# Stage 3: Final deployment image
FROM base AS deployment

WORKDIR ${ADT4_WS}

# Copy the built workspace, source, and build artifacts from the builder stage.
COPY --from=builder ${ADT4_WS}/install ./install
COPY --from=builder ${ADT4_WS}/src ./src
COPY --from=builder ${ADT4_WS}/build ./build
COPY --from=builder ${ADT4_ENV} ${ADT4_ENV}
COPY --from=builder /usr/local/bin/fast-downward /usr/local/bin/fast-downward