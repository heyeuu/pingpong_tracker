# ====================================================
# 阶段 1: builder - 你的开发环境
# ====================================================
FROM ros:humble-ros-base AS builder

ENV TZ=Asia/Shanghai \
    DEBIAN_FRONTEND=noninteractive

USER root

# 安装所有开发和构建工具，以及项目依赖
# RUN sed -i 's/archive.ubuntu.com/mirrors.tuna.tsinghua.edu.cn/g' /etc/apt/sources.list && \
#     sed -i 's/security.ubuntu.com/mirrors.tuna.tsinghua.edu.cn/g' /etc/apt/sources.list  
RUN apt-get update && apt-get install -y --no-install-recommends software-properties-common wget gnupg && \
    sudo add-apt-repository universe && \
    sudo wget https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -O /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://mirrors.tuna.tsinghua.edu.cn/ros2/ubuntu/ $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

RUN  apt-get update && apt-get upgrade -y && apt-get install -y --no-install-recommends \
    # 基础系统工具 (为了开发方便)
    sudo fish zsh vim wget gnupg ca-certificates unzip net-tools iputils-ping ripgrep htop fzf gnutls-bin\
    usbutils \
    # 开发与构建工具
    build-essential ninja-build libc6-dev git \
    # ROS 2 核心工具
    python3-colcon-common-extensions python3-rosdep python3-colorama \
    ros-humble-launch-ros \
    ros-humble-sensor-msgs \ 
    ros-humble-rviz2 ros-humble-rqt ros-humble-rqt-common-plugins ros-humble-rqt-image-view ros-humble-foxglove-bridge \
    # 相机驱动与图像处理的核心依赖
    libusb-1.0-0-dev \
    libopencv-dev\
    ros-humble-camera-info-manager \
    ros-humble-camera-calibration \
    ros-humble-cv-bridge ros-humble-image-transport ros-humble-image-transport-plugins ros-humble-image-tools \
    # 开发辅助工具
    clangd-15 python3-dpkt software-properties-common \
    && apt-get clean && rm -rf /var/lib/apt/lists/*

# 创建非 root 用户
ARG USERNAME=developer
ARG USER_UID=1000
ARG USER_GID=$USER_UID

RUN groupadd --gid $USER_GID $USERNAME && \
    useradd --uid $USER_UID --gid $USER_GID -m $USERNAME && \
    usermod -aG sudo $USERNAME && \
    echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME && \
    chmod 0440 /etc/sudoers.d/$USERNAME

USER $USERNAME
# 设置工作空间路径
WORKDIR /workspaces/pingpong_tracker/pingpong_tracker_ws

# Install oh my zsh, change theme to af-magic and setup environment of zsh
RUN sh -c "$(wget https://raw.githubusercontent.com/ohmyzsh/ohmyzsh/master/tools/install.sh -O -)" && \
    sed -i 's/ZSH_THEME=\"[a-z0-9\-]*\"/ZSH_THEME="af-magic"/g' ~/.zshrc && \
    echo 'export PATH=${PATH}:/workspaces/pingpong_tracker/.scripts' >> ~/.zshrc

# CMD ["/bin/bash", "-c"]