# 使用 ROS Humble 基础镜像
FROM ros:humble-ros-base

# 切换为 root 用户以安装软件包
USER root

# 更新并安装所有必要的依赖
RUN apt-get update && apt-get install -y \
    # 安装终端和通用开发工具 
    fish \
    build-essential \
    ninja-build \
    git \
    sudo \
    # ROS 构建工具
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-colorama python3-dpkt \
    # 项目的系统依赖
    libusb-1.0-0-dev \
    #安装特定版本的Clangd和GCC(TODO)先把版本写死
    clangd-14 \
    # 你的ROS包及其依赖
    ros-humble-sensor-msgs \
    ros-humble-cv-bridge \
    ros-humble-rqt \
    ros-humble-rqt-common-plugins \
    ros-humble-rqt-image-view \
    ros-humble-image-transport \
    ros-humble-image-transport-plugins \
    ros-humble-camera-info-manager \
    ros-humble-camera-calibration \
    ros-humble-image-tools \
    ros-humble-launch-ros \
    && rm -rf /var/lib/apt/lists/*

# 创建非 root 用户
ARG USERNAME=developer
ARG USER_UID=1000
ARG USER_GID=$USER_UID

# 添加用户组并创建用户
RUN groupadd --gid $USER_GID $USERNAME && \
    useradd --uid $USER_UID --gid $USER_GID -m $USERNAME && \
    # 允许该用户使用 sudo 且无需密码。
    # 这将创建一个文件，告诉 sudoers 规则允许此用户无密码运行任何命令。
    echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME && \
    chmod 0440 /etc/sudoers.d/$USERNAME

# 切换到非 root 用户
USER $USERNAME
# 设置工作目录
WORKDIR /workspaces/pingpong_tracker
# 设置入口点为 bash，以便容器启动后进入交互式 shell
CMD ["/bin/bash"]