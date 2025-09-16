#!/bin/bash

set -e  

echo "installing the dependencies..."

sudo mkdir -p /etc/apt/keyrings/
curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo gpg --dearmor | sudo tee /etc/apt/keyrings/ros2.gpg > /dev/null
wget -qO - https://apt.llvm.org/llvm.sh | sudo bash -s -- 22

sudo add-apt-repository -y ppa:ubuntu-toolchain-r/test
sudo apt-get install gnupg
wget https://apt.repos.intel.com/intel-gpg-keys/GPG-PUB-KEY-INTEL-SW-PRODUCTS.PUB
sudo gpg --output /etc/apt/trusted.gpg.d/intel.gpg --dearmor GPG-PUB-KEY-INTEL-SW-PRODUCTS.PUB
rm GPG-PUB-KEY-INTEL-SW-PRODUCTS.PUB
echo "deb https://apt.repos.intel.com/openvino ubuntu22 main" | sudo tee /etc/apt/sources.list.d/intel-openvino.list
sudo apt update
sudo apt install -y \
    g++ \
    cmake \
    libopencv-dev \
    libfmt-dev \
    libeigen3-dev \
    libspdlog-dev \
    libyaml-cpp-dev \
    libusb-1.0-0-dev \
    nlohmann-json3-dev \
    openssh-server \
    screen \
    openvino\
    gcc-13 g++-13 \
    libtbb-dev \
    libceres-dev \
    libdwarf-dev libbackward-cpp-dev\
    binutils-dev  libdw-dev \
    clangd-22

sudo ln -sf /usr/bin/clangd-22 /usr/local/bin/clangd
# sudo ln -s /usr/include/libdwarf/libdwarf.h /usr/include/libdwarf.h 
# sudo ln -s /usr/include/libdwarf/dwarf.h /usr/include/dwarf.h

sudo update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-13 13
sudo update-alternatives --install /usr/bin/g++ g++ /usr/bin/g++-13 13
sudo update-alternatives --config g++
sudo update-alternatives --config gcc

echo "dependencies installation complished!"