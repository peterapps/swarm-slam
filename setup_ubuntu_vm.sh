#!/bin/bash

TMP_DIR=$(pwd)

# VMware Tools Shared Folders Linux mounts
sudo apt install open-vm-tools open-vm-tools-desktop
sudo sh -c "echo \"vmhgfs-fuse    /mnt/hgfs    fuse    defaults,allow_other    0    0\" >> /etc/fstab"

# Install ROS 2 Foxy
sudo apt update
sudo apt install -y --no-install-recommends dirmngr gnupg2 lsb-release

sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

sudo apt install software-properties-common
sudo add-apt-repository universe

sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

sudo apt update
sudo apt upgrade
sudo apt install ros-foxy-desktop python3-argcomplete
sudo apt install ros-dev-tools

# Install Swarm-SLAM dependencies
sudo apt update
sudo apt upgrade -y
sudo apt install --no-install-recommends -y \
    bash-completion \
    python3-pip \
    libboost-all-dev \
    software-properties-common \
    ros-foxy-rtabmap-ros \
    ros-foxy-pcl-ros \
    ros-foxy-perception-pcl

# Install gtsam
sudo add-apt-repository -y ppa:borglab/gtsam-release-4.1
sudo apt install  --no-install-recommends -y libgtsam-dev libgtsam-unstable-dev

# Install pip dependencies
pip3 install -r requirements.txt

# There seems to be a version issue with numba
pip3 install --upgrade numba

# Install TEASER++
cd ~/Documents
git clone https://github.com/MIT-SPARK/TEASER-plusplus.git
cd TEASER-plusplus
mkdir build
cd build
cmake ..
make
sudo make install
sudo ldconfig
cmake -DTEASERPP_PYTHON_VERSION=3.8 ..
make teaserpp_python
cd python
pip3 install .

# SSC
cd ~/Documents
sudo apt install libpcl-dev libyaml-cpp-dev
git clone https://github.com/lilin-hitcrt/SSC.git
cd SSC
mkdir build
cd build
cmake ..
make

# Miscellaneous dependencies
pip3 install distinctipy
pip3 install numpy-quaternion
sudo apt install -y python3-pcl pcl-tools
pip3 install rosbags

echo "source ~/rob530_setup.bash" >> ~/.bashrc

cd $TMP_DIR