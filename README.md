# pingpong_tracker
这是一个基于 ROS 2 Humble 的计算机视觉项目，旨在利用 MindVision 工业相机实时追踪乒乓球的位置。该项目以 docker container 形式提供，旨在为开发者提供一个即开即用、完全可复现的开发环境

Continuously updating...

## 目录
- [项目概览](#项目概览)
- [技术栈](#技术栈)
- [项目结构](#项目结构)
- [快速开始](#快速开始)
- [工作流程](#工作流程)
- [许可证](#许可证)

## 项目概览
本项目包含一个 ROS 2 工作空间`pingpong_tracker_ws`，用于实现基于视觉的乒乓球追踪功能。其核心组件包括：
1.  **`pingpong_tracker_ws/src/ros2_mindvision_camera`**：一个 ROS 2 包，作为与 MindVision 工业相机交互的驱动。它负责从相机捕获图像，并将图像数据发布到 ROS 2 网络上。

2.  **Dev Container 环境**：项目配置了 Dockerfile 和 devcontainer.json，提供一个预配置好的 Ubuntu 22.04 + ROS 2 Humble 环境，确保所有依赖项和工具链都已安装。

TODO

## 技术栈

-   **ROS 2 Humble**：核心机器人框架。
-   **Docker**：用于创建隔离的、可复现的开发环境。
-   **C++**：主要编程语言，用于高性能的图像处理节点。
-   **Python**：用于编写 ROS 2 启动文件和脚本。
-   **OpenCV**：强大的计算机视觉库，用于图像处理和球体检测。TODO
-   **MindVision SDK**：用于与 MindVision 工业相机进行通信。
-   **TODO**:


## 项目结构
pingpong_tracker
├── .devcontainer/                  # Dev Container 配置目录
│   └── devcontainer.json           # VS Code Dev Container 配置文件
├── .scripts/                       # 自动化脚本目录
│   ├── build-pingpong.sh           # 构建 ROS 2 工作空间的脚本
│   └── init_devcontainer.sh        # 初始化容器环境的脚本
├── Dockerfile                      # Dev Container 的 Dockerfile
├── pingpong_tracker_ws/            # ROS 2 工作空间
│   └── src/                        # 源码目录
│       ├── ros2_mindvision_camera/ # Git 子模块，MindVision 相机驱动
│       └── TODO  # RODO
└── README.md                       # 本文件

## 快速开始

### 前置条件
-   安装 [Git](https://git-scm.com/)。
-   安装 [Docker Desktop](https://www.docker.com/products/docker-desktop/)（适用于 Windows/macOS）或 [Docker Engine](https://docs.docker.com/engine/install/)(适用于 Linux)。TODO
-   安装 [Visual Studio Code](https://code.visualstudio.com/)。
-   安装 VS Code 的 [Dev Containers 扩展](https://marketplace.visualstudio.com/items?itemName=ms-vscode-remote.remote-containers)。
TODO

### Step 1：克隆并打开仓库
克隆仓库，注意需要使用 `recurse-submodules` 以克隆子模块：

```bash
#克隆本仓库,初次克隆会自动克隆子模块
git clone https://github.com/heyeuu/pingpong_tracker.git
#克隆子模块
git clone --recurse-submodules https://github.com/heyeuu/pingpong_tracker.git
```
在VSCode中打开仓库:
```bash
code ./pingpong_tracker
```
### Step 1：获取镜像并进入容器
#### method one:
可自行使用 `Dockerfile` 构建，参见 [镜像构建指南](docs/zh-cn/build_docker_image.md),按 `Ctrl+Shift+P`，在弹出的菜单中选择 `Dev Containers: Reopen in Container`。

#### method two:
按 `Ctrl+Shift+P`，在弹出的菜单中选择 `Dev Containers: Rebuild and reopen in Container`

VSCode 将拉起一个 `Docker` 容器，容器中已配置好完整开发环境，之后所有工作将在容器内进行。

如果 `Dev Containers` 在启动时卡住很长一段时间，可以尝试 [这个解决方案](docs/zh-cn/fix_devcontainer_stuck.md)。

### Step 3：配置 VSCode

在 VSCode 中新建终端，输入：

```bash
cp .vscode/settings.default.json .vscode/settings.json
```

这会应用我们推荐的 VSCode 配置文件，你也可以按需自行修改配置文件。

在拓展列表中，可以看到我们推荐使用的拓展正在安装，你也可以按需自行删减拓展。

### Step 4：构建

在 VSCode 终端中输入：

TODO

目前你可以手动运行`./.scripts/build-pingpong.sh`脚本，将在路径 `pingpong_tracker_ws` 下开始构建代码。
```
./.scripts/build-pingpong.sh
```
构建完毕后，基于 `clangd` 的 `C++` 代码提示将可用。此时可以正常编写代码。

### Step 5 (Optional)：运行

编写代码并编译完成后，可以使用：

```bash
# 在运行 ROS 2 程序前，请先配置环境
source pingpong_tracker_ws/install/setup.bash

# 使用 Launch 文件启动完整的系统
TODO
```
在本机上运行代码
(ros包，所以ros编译运行的那一套也能用)
(你可以使用)
```
TODO
```
#### 确认设备接入

可以使用 `lsusb` 确定 [下位机](https://github.com/Alliance-Algorithm/rmcs_slave) 是否已接入，若已接入，则 `lsusb` 输出类似：

```
TODO
Bus 001 Device 004: ID a11c:75f3 Alliance RoboMaster Team. RMCS Slave v2.1.2
TODO
```
TODOTODOTODO

## 工作流程

1.    在 Dev Container 中修改代码。

2.   每次修改后，运行 ./.scripts/build-pingpong.sh 重新编译。

3.    如果需要运行，执行 source 命令并使用 ros2 launch 启动节点。

4.  使用 git add、git commit 和 git push 将你的更改同步到远程仓库。

## 许可证
本项目使用 **MIT 许可证**。请查看 [LICENSE](LICENSE) 文件了解更多详情。