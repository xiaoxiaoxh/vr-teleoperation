FROM osrf/ros:humble-desktop

WORKDIR /root/teleop-env
SHELL ["/bin/bash", "-c"]

COPY third_party third_party
COPY requirements.txt requirements.txt

RUN apt-get update && \
    apt-get install -y ros-humble-tf-transformations python3-venv python3-pip tmux v4l-utils usbutils wget && \
    apt-get clean

RUN python3 -m venv venv

# Install pytorch
RUN . venv/bin/activate && \
    pip install --upgrade pip && \
    pip install torch torchvision torchaudio --index-url https://download.pytorch.org/whl/cu121

# Install CUDA
# Note that we use conda only for installing CUDA, and we don't use it for managing the environment because it's not compatible with ROS2
ARG MINICONDA_VERSION=latest
RUN wget https://repo.anaconda.com/miniconda/Miniconda3-${MINICONDA_VERSION}-Linux-x86_64.sh -O miniconda.sh && \
    bash miniconda.sh -b -p /opt/miniconda && \
    rm miniconda.sh
ARG CUDA_VERSION=12.2.1
RUN export PATH=/opt/miniconda/bin:$PATH && \
    conda create -n cuda_env -y nvidia/label/cuda-${CUDA_VERSION}::cuda-toolkit && \
    echo "export PATH=/opt/miniconda/envs/cuda_env/bin:$PATH" >> ~/.bashrc

# Install pytorch3d_simplified
RUN . venv/bin/activate && \
    cd third_party/pytorch3d_simplified && \
    python setup.py install

# Install other requirements
RUN . venv/bin/activate && \
    pip install -r requirements.txt

# Install ros2_numpy_humble
RUN . venv/bin/activate && \
    cd third_party/ros2_numpy_humble && \
    pip install -e .