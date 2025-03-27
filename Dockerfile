FROM pytorch/pytorch:2.6.0-cuda12.6-cudnn9-devel

LABEL org.opencontainers.image.source="https://github.com/iKrishneel/foundation_stereo_ros/" \
    org.opencontainers.image.description="Foundation Stereo ROS" \
    org.opencontainers.image.version="0.0.1"

ENV LANG=en_US.UTF-8
ENV DEBIAN_FRONTEND=noninteractive
ENV NVIDIA_VISIBLE_DEVICES=all

SHELL ["/bin/bash", "-c"]

RUN apt update
RUN apt install -y git software-properties-common curl
RUN add-apt-repository universe && \
    curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key | tee /usr/share/keyrings/ros-archive-keyring.gpg > /dev/null && \
    echo "deb [signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null && \
    apt update && apt install -y ros-humble-desktop python3-colcon-common-extensions python3-rosdep python3-vcstool && \
    rosdep init && rosdep update && \
    echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc

RUN apt install -y build-essential cmake ros-humble-ament-cmake
RUN rm -rf /var/lib/apt/lists/*

WORKDIR /root/ros/humble/
COPY . ./src/foundation_stereo_ros/

# RUN pip install catkin_pkg && pip install -r ./src/foundation_stereo_ros/requirements.txt

RUN rosdep install -ryi --from-paths ./src/
RUN source /opt/ros/humble/setup.bash && \
    colcon build --symlink-install --event-handlers console_direct+ --packages-up-to foundation_stereo_ros

# RUN git clone https://github.com/NVlabs/FoundationStereo.git
