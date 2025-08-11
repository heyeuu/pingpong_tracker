# pingpong_tracker
这是一个基于 ROS 2 Humble 的计算机视觉项目，旨在利用 MindVision 工业相机实时追踪乒乓球的位置。该项目以 docker container 形式提供，旨在为开发者提供一个即开即用、完全可复现的开发环境

Continuously updating...

## 目录
- [项目结构](#项目结构)
- [快速开始](#快速开始)
- [工作流程](#工作流程)
- [许可证](#许可证)

## 项目结构
```
pingpong_tracker_ws/
├── build/                 # 由colcon自动生成
├── install/               # 由colcon自动生成
├── log/                   # 由ROS 2自动生成
└── src/
    ├── camera_driver/       # 包1：摄像头驱动模块**
    │   ├── include/
    │   │   └── camera_driver/
    │   │       └── mv_camera_component.hpp      # Composable Node 类定义
    │   ├── src/
    │   │   ├── mv_camera_component.cpp          # Composable Node 类实现
    │   │   └── mv_camera_standalone_main.cpp    # 独立运行的main函数
    │   ├── CMakeLists.txt
    │   └── package.xml
    │
    ├── vision_pipeline/     # 包2：视觉处理模块**
    │   ├── include/
    │   │   └── vision_pipeline/
    │   │       ├── deep_learning_component.hpp  # Composable Node 类定义
    │   │       ├── kalman_filter_component.hpp  # Composable Node 类定义
    │   │       └── shared_data_types.hpp        # 共享的数据结构
    │   ├── src/
    │   │   ├── deep_learning_component.cpp      # 深度学习组件类实现
    │   │   ├── kalman_filter_component.cpp      # 卡尔曼滤波组件类实现
    │   │   ├── dl_standalone_main.cpp           # 独立运行的main函数
    │   │   └── kf_standalone_main.cpp           # 独立运行的main函数
    │   ├── models/
    │   │   └── yolov8.onnx
    │   ├── CMakeLists.txt
    │   └── package.xml
    │
    └── pingpong_tracker_bringup/  # 包3：系统启动和配置模块**
        ├── launch/
        │   ├── debug_dl.launch.py               # 用于单独调试深度学习的launch文件
        │   ├── debug_kf.launch.py               # 用于单独调试卡尔曼滤波的launch文件
        │   └── tracker.launch.py                # 用于高性能部署的launch文件
        └── config/
            ├── camera_params.yaml
            ├── dl_params.yaml
            └── kf_params.yaml
        ├── CMakeLists.txt
        └── package.xml
```
## 快速开始

### 前置条件
-   安装 [Git](https://git-scm.com/)。
-   安装 [Docker] TODO
-   安装 [Visual Studio Code](https://code.visualstudio.com/)。
-   安装 VS Code 的 [Dev Containers 扩展](https://marketplace.visualstudio.com/items?itemName=ms-vscode-remote.remote-containers)。
TODO

### Step 1：克隆并打开仓库
克隆仓库，注意需要使用 `recurse-submodules` 以克隆子模块：

```
#克隆本仓库,初次克隆会自动克隆子模块
git clone https://github.com/heyeuu/pingpong_tracker.git
#克隆子模块
git clone --recurse-submodules https://github.com/heyeuu/pingpong_tracker.git
```
在VSCode中打开仓库:
```
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

你可以在终端中输入
```
./.scripts/脚本的名称
(e.g.build.sh)
```
（TODO，好像没有很优雅）

也可以手动运行`./.scripts/build-pingpong.sh`脚本，将在路径 `pingpong_tracker_ws` 下开始构建代码。
```
./.scripts/build-pingpong.sh
```
构建完毕后，基于 `clangd` 的 `C++` 代码提示将可用。此时可以正常编写代码。

### Step 5 (Optional)：运行

编写代码并编译完成后，可以使用：

```zsh
launch.sh
```
运行     （好像不优雅）TODO
#### 确认设备接入

可以使用 `lsusb`  确定相机是否已接入，若已接入，则 `lsusb` 输出类似：

```
Bus 004 Device 004: ID f622:0001 MindVision SUA133GC
```

在<span style="color:red; background-color:#FFFF00; font-weight:bold">本机 </span>输入以下命令来确保你可以从容器中访问usb设备:
```
echo 'SUBSYSTEM=="usb", ATTR{idVendor}=="f622", ATTR{idProduct}=="0001", MODE="0666", GROUP="plugdev"' | sudo tee /etc/udev/rules.d/95-mindvision.rules && 
sudo udevadm control --reload-rules && sudo udevadm trigger
```
TODOTODOTODO

##标定方法
###单目
运行相机节点后再开一个终端运行camera_calibration节点,在终端中输入:
```
source pingpong_tracker_ws/install/setup.zsh
ros2 launch mindvision_camera mv_launch.py

ros2 run <your_camera_package> <your_camera_node>
(e.g.)
ros2 run camera_calibration cameracalibrator --ros-args --remap image:=/image_raw -- --size 7x5 --square 0.03

```
###双目
TODO

```
source pingpong_tracker_ws/install/setup.zsh
ros2 launch mindvision_camera mv_launch.py

ros2 run camera_calibration cameracalibrator --ros-args --remap left:=/my_stereo/left/image_raw --remap right:=/my_stereo/right/image_raw -- --size <NxM> --square <size_in_meters>

```
## 工作流程

1.    在 Dev Container 中修改代码。

2.   每次修改后，运行 ./.scripts/build-pingpong.sh 重新编译。

3.    如果需要运行，执行 source 命令并使用 ros2 launch 启动节点。

4.  使用 git add、git commit 和 git push 将你的更改同步到远程仓库。

## TODO LIST

- [ ] 把整体的框架搭建出来
- [ ] 整理dockerfile，在一个base image的上构建develop image 和 deploy image，并筛选移除多余依赖
- [ ] ros环境导入自动化
- [ ] 搭建完框架再说

---

## 许可证
本项目使用 **MIT 许可证**。请查看 [LICENSE](LICENSE) 文件了解更多详情。