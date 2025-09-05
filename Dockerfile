
FROM ubuntu:22.04

ENV TZ=Asia/Shanghai \
    DEBIAN_FRONTEND=noninteractive

USER root

RUN apt-get update && \
    apt-get install -y --no-install-recommends \
    sudo \
    curl \
    wget \
    gnupg \
    ca-certificates \
    zsh \
    git \
    software-properties-common && \
    rm -rf /var/lib/apt/lists/*

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

WORKDIR /workspaces/pingpong_tracker/pingpong_tracker_ws

# Install oh my zsh, change theme to af-magic and setup environment of zsh
RUN sh -c "$(wget https://raw.githubusercontent.com/ohmyzsh/ohmyzsh/master/tools/install.sh -O -)" && \
    sed -i 's/ZSH_THEME=\"[a-z0-9\-]*\"/ZSH_THEME="af-magic"/g' ~/.zshrc && \
    echo 'export PATH=${PATH}:/workspaces/pingpong_tracker/.scripts' >> ~/.zshrc 
