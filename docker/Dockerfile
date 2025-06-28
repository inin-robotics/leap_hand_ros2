FROM osrf/ros:humble-desktop

ARG DEBIAN_FRONTEND=noninteractive
ARG USER_UID
ARG USER_GID
ARG USERNAME

ENV LANG=C.UTF-8 \
    LC_ALL=C.UTF-8 \
    ROS_DISTRO=humble

# Install essential packages and ROS development tools
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
        bash-completion \
        curl \
        gdb \
        git \
        openssh-client \
        python3-colcon-argcomplete \
        python3-colcon-common-extensions \
        sudo \
        vim \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*

# Setup user configuration
RUN groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME \
    && echo "$USERNAME ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers \
    && echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> /home/$USERNAME/.bashrc \
    && echo "source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash" >> /home/$USERNAME/.bashrc

# switch DDS implementation to zenoh
ENV RMW_IMPLEMENTATION=rmw_zenoh_cpp
RUN apt-get update \
    && apt-get install -y ros-$ROS_DISTRO-rmw-zenoh-cpp \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*

USER $USERNAME

# Dependencies for LEAP hand ros2 driver
RUN sudo apt-get update \
    && sudo apt-get install -y --no-install-recommends \
        python3-serial \
        ros-${ROS_DISTRO}-dynamixel-sdk \
    && sudo apt-get clean \
    && sudo rm -rf /var/lib/apt/lists/*
# add user to dialout group for serial port access
RUN sudo usermod -aG dialout $USERNAME

ARG IDE_TYPE
# make workspace dir and ide-specific dir. note that .vscode work for both vscode and cursor
RUN mkdir /home/${USERNAME}/.$IDE_TYPE-server \
    && mkdir -p /home/${USERNAME}/ros2_ws/.vscode \
    && mkdir /home/${USERNAME}/ros2_ws/src

WORKDIR /home/$USERNAME/ros2_ws
