# FROM pytorch/pytorch:2.6.0-cuda12.6-cudnn9-devel
FROM pytorch/pytorch:2.5.0-cuda12.1-cudnn9-devel

LABEL org.opencontainers.image.source="https://github.com/iKrishneel/foundation_stereo_ros/" \
    org.opencontainers.image.description="Foundation Stereo ROS" \
    org.opencontainers.image.version="0.0.1"

ENV LANG=en_US.UTF-8
ENV DEBIAN_FRONTEND=noninteractive
ENV NVIDIA_VISIBLE_DEVICES=all
ENV ROS_DISTRO=humble

SHELL ["/bin/bash", "-c"]
WORKDIR /root/ros/$ROS_DISTRO/

RUN apt update && apt install -y git git-lfs software-properties-common curl
RUN git clone https://github.com/iKrishneel/foundation_stereo_ros.git ./src/foundation_stereo_ros && \
    cd src/foundation_stereo_ros/weights/ && git submodule update --init --recursive --force && ./post_clone.sh
RUN add-apt-repository universe && \
    curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key | tee /usr/share/keyrings/ros-archive-keyring.gpg > /dev/null && \
    echo "deb [signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null && \
    apt update && apt install -y ros-$ROS_DISTRO-desktop python3-colcon-common-extensions python3-rosdep python3-vcstool build-essential cmake ros-$ROS_DISTRO-ament-cmake && \
    rosdep init && rosdep update

RUN rosdep install -ryi --from-paths ./src/ && \
    pip install catkin_pkg && \
    source /opt/ros/$ROS_DISTRO/setup.bash && \
    colcon build --symlink-install --event-handlers console_direct+ --packages-up-to foundation_stereo_ros && \
    echo "source /root/ros/$ROS_DISTRO/install/setup.bash" >> ~/.bashrc && \
    rm -rf /var/lib/apt/lists/*
