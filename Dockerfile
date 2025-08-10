# 使用 ROS Humble 基础镜像
FROM ros:humble-ros-base

# Set timezone and non-interactive mode
ENV TZ=Asia/Shanghai \
    DEBIAN_FRONTEND=noninteractive
# 切换为 root 用户以安装软件包
USER root

# ----------------------------------------------------
# 第1步：安装通用依赖和构建工具
# ----------------------------------------------------
RUN apt-get update && apt-get upgrade -y && apt-get install -y --no-install-recommends \
    # ===== 1. 基础系统工具 =====
    sudo \
    usbutils \
    fish \
    vim \
    wget \
    gnupg \
    ca-certificates \
    # ===== 2. 开发与构建工具 =====
    build-essential \
    ninja-build \
    libc6-dev \
    git \
    # ===== 3. ROS 2 核心工具 =====
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-colorama \
    ros-humble-launch-ros \
    ros-humble-sensor-msgs \
    # ===== 4. 相机与图像处理 =====
    libusb-1.0-0-dev \
    ros-humble-camera-info-manager \
    ros-humble-camera-calibration \
    ros-humble-cv-bridge \
    ros-humble-image-transport \
    ros-humble-image-transport-plugins \
    ros-humble-image-tools \
    # ===== 5. GUI 与调试工具 =====
    ros-humble-rqt \
    ros-humble-rqt-common-plugins \
    ros-humble-rqt-image-view \
    # ===== 6. 开发辅助工具=====
    clangd-15 \
    python3-dpkt \
    software-properties-common

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