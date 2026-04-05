# dcist-dev: Development image with rosdep deps, Python venvs, and dev tools
# Built on top of dcist-base. Workspace source is bind-mounted at runtime.
#
# Usage:
#   docker build -f docker/dev.Dockerfile -t dcist-dev:latest \
#     --build-arg BASE_IMAGE=dcist-base:latest ..
#
# The build context should be the awesome_dcist_t4 directory.
# Submodules must be checked out on the host before building.

ARG BASE_IMAGE=dcist-base:latest
FROM ${BASE_IMAGE}

ENV DEBIAN_FRONTEND=noninteractive
ENV ADT4_WS=/root/dcist_ws
ENV ADT4_ENV=/opt/venvs

# ---- rosdep layer ----
# Copy the full source tree temporarily for rosdep resolution, then clean up.
# This layer is invalidated when package.xml files change.
COPY . /tmp/deps/src/awesome_dcist_t4/
RUN apt-get update && \
    rosdep install --from-paths /tmp/deps/src --ignore-src -r -y --rosdistro=${ROS_DISTRO} || true && \
    rm -rf /tmp/deps /var/lib/apt/lists/*

# ---- Python venv layer: ROMAN ----
# We replicate what python_setup.bash --no-spark does, but without git submodule
# commands (the submodule contents are included via COPY from the host).
COPY install/roman_requirements.txt /tmp/reqs/roman_requirements.txt
COPY roman /tmp/roman_src/

RUN python3 -m venv ${ADT4_ENV}/roman && \
    ${ADT4_ENV}/roman/bin/pip install --no-cache-dir --upgrade pip && \
    # Build CLIPPER (roman submodule dependency -- already checked out on host)
    # Clean any stale CMakeCache from host build
    rm -rf /tmp/roman_src/dependencies/clipper/build && \
    mkdir -p /tmp/roman_src/dependencies/clipper/build && \
    cd /tmp/roman_src/dependencies/clipper/build && \
    cmake .. -DPYTHON_EXECUTABLE=${ADT4_ENV}/roman/bin/python && \
    make && make pip-install && \
    # Install roman in editable mode + requirements
    cd /tmp/roman_src && \
    ${ADT4_ENV}/roman/bin/pip install --no-cache-dir -e . --no-deps && \
    ${ADT4_ENV}/roman/bin/pip install --no-cache-dir -r /tmp/reqs/roman_requirements.txt && \
    # FastSAM
    ${ADT4_ENV}/roman/bin/pip install --no-cache-dir --no-build-isolation \
      git+https://github.com/CASIA-IVA-Lab/FastSAM.git@4d153e9 && \
    # Download roman model weights
    mkdir -p /tmp/weights && \
    ${ADT4_ENV}/roman/bin/gdown 'https://drive.google.com/uc?id=1m1sjY4ihXBU1fZXdQ-Xdj-mDltW-2Rqv' -O /tmp/weights/FastSAM-x.pt && \
    ([ -f /tmp/weights/yolov7.pt ] || wget -P /tmp/weights https://github.com/WongKinYiu/yolov7/releases/download/v0.1/yolov7.pt) && \
    # Remove bundled NVIDIA CUDA packages — the base image already has system CUDA.
    # PyTorch will use system CUDA libs via LD_LIBRARY_PATH instead.
    ${ADT4_ENV}/roman/bin/pip uninstall -y \
      nvidia-cublas nvidia-cuda-cupti nvidia-cuda-nvrtc nvidia-cuda-runtime \
      nvidia-cudnn-cu13 nvidia-cufft nvidia-cufile nvidia-curand nvidia-cusolver \
      nvidia-cusparse nvidia-cusparselt-cu13 nvidia-nccl-cu13 nvidia-nvjitlink \
      nvidia-nvshmem-cu13 nvidia-nvtx triton 2>/dev/null || true && \
    rm -rf /tmp/roman_src /tmp/reqs

# ---- Python venv layer: spark_env ----
COPY install/spark_requirements.txt /tmp/reqs/spark_requirements.txt
COPY semantic_inference/semantic_inference /tmp/semantic_inference/

RUN python3 -m venv ${ADT4_ENV}/spark_env --system-site-packages && \
    ${ADT4_ENV}/spark_env/bin/pip install --no-cache-dir --upgrade pip && \
    ${ADT4_ENV}/spark_env/bin/pip install --no-cache-dir -r /tmp/reqs/spark_requirements.txt && \
    ${ADT4_ENV}/spark_env/bin/pip install --no-cache-dir -e /tmp/semantic_inference && \
    rm -rf /tmp/reqs /tmp/semantic_inference

# ---- Spark model weights ----
COPY install/download_weights.py /tmp/download_weights.py
RUN mkdir -p /tmp/weights && \
    ${ADT4_ENV}/spark_env/bin/python /tmp/download_weights.py \
      -d /tmp/weights \
      yolov8s-world.pt yoloe-26m-seg.pt mobileclip2_b.ts || true && \
    rm -f /tmp/download_weights.py

# Note: Docker named volumes auto-initialize from the image contents on first use.
# No backup copy needed — /opt/venvs content is copied to the volume automatically.

# ---- fast-downward ----
RUN git clone https://github.com/aibasel/downward.git /opt/fast_downward && \
    cd /opt/fast_downward && \
    python3 build.py && \
    ln -sf /opt/fast_downward/fast-downward.py /usr/local/bin/fast-downward && \
    touch /opt/fast_downward/COLCON_IGNORE

# ---- zsh + oh-my-zsh ----
RUN apt-get update && apt-get install -y zsh && rm -rf /var/lib/apt/lists/* && \
    sh -c "$(curl -fsSL https://raw.githubusercontent.com/ohmyzsh/ohmyzsh/master/tools/install.sh)" "" --unattended && \
    chsh -s /usr/bin/zsh root

# ---- Shell configuration (.zshrc) ----
RUN cat >> /root/.zshrc <<'ZSHEOF'

# ROS 2
source /opt/ros/jazzy/setup.zsh
[ -f /root/dcist_ws/install/setup.zsh ] && source /root/dcist_ws/install/setup.zsh

# Aliases
alias cb='cd /root/dcist_ws && colcon build --symlink-install --continue-on-error'
alias roman='source /opt/venvs/roman/bin/activate'
alias spark='source /opt/venvs/spark_env/bin/activate'

# Environment
export ADT4_WS=/root/dcist_ws
export ADT4_ENV=/opt/venvs
export PATH="/root/.local/bin:${PATH}"
ZSHEOF

WORKDIR /root/dcist_ws

COPY docker/scripts/entrypoint-dev.sh /entrypoint-dev.sh
RUN chmod +x /entrypoint-dev.sh

ENTRYPOINT ["/entrypoint-dev.sh"]
CMD ["zsh"]
