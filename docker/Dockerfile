FROM osrf/ros:humble-desktop

WORKDIR /root/teleop-env

COPY third_party third_party
COPY requirements.txt requirements.txt

RUN apt-get update && \
    apt-get install -y ros-humble-tf-transformations python3-venv python3-pip tmux v4l-utils usbutils&& \
    apt-get clean

RUN python3 -m venv venv

# Install pytorch
RUN . venv/bin/activate && \
    pip install --upgrade pip && \
    pip install torch torchvision torchaudio --index-url https://download.pytorch.org/whl/cu121

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