#!/bin/bash

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
git clone https://github.com/MIT-SPARK/TEASER-plusplus.git
cd TEASER-plusplus
mkdir build
cd build
cmake ..
make
make install
ldconfig
cmake -DTEASERPP_PYTHON_VERSION=3.8 ..
make teaserpp_python
cd python
pip3 install .

tee -a ~/.bashrc <<EOF
alias sros1="source /opt/ros/noetic/setup.bash"
alias sros2="source /opt/ros/foxy/setup.bash"
alias sws="source ~/ros_ws/install/setup.bash"
alias cdws="cd ~/ros_ws"

buildws(){
    TMP_DIR=\$(pwd)
    cdws
    sros2
    colcon build --symlink-install && sws
    cd $TMP_DIR
}
EOF

echo "alias sros1=\"source /opt/ros/noetic/setup.bash\"" >> /root/.bashrc
echo "alias sros2=\"source /opt/ros/foxy/setup.bash\"" >> /root/.bashrc
echo "alias sws=\"source /ros_ws/install/setup.bash\"" >> /root/.bashrc
echo "sros2" >> /root/.bashrc
echo "test -f /ros_ws/install/setup.bash && sws" >> /root/.bashrc