FROM fredblgr/ubuntu-novnc:20.04

LABEL Name=swarmslam Version=0.0.2

ENV DEBIAN_FRONTEND noninteractive

# Setup timezone for ROS
RUN echo 'US/Eastern' > /etc/timezone && \
    ln -s /usr/share/zoneinfo/US/Eastern /etc/localtime && \
    apt-get update && \
    apt-get install -q -y --no-install-recommends tzdata && \
    rm -rf /var/lib/apt/lists/*

# Install packages
RUN apt-get update && apt-get install -q -y --no-install-recommends \
    dirmngr \
    gnupg2 \
    && rm -rf /var/lib/apt/lists/*

# Setup sources.list
RUN echo "deb http://packages.ros.org/ros2/ubuntu focal main" > /etc/apt/sources.list.d/ros2-latest.list

# Setup keys
RUN apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-keys C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654

# Setup environment
ENV LANG C.UTF-8
ENV LC_ALL C.UTF-8

# Install ROS 2 packages
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-foxy-desktop \
    python3-argcomplete \
    ros-dev-tools \
    && rm -rf /var/lib/apt/lists/*

# Install Swarm-SLAM dependencies
RUN apt update && \
    apt upgrade -y && \
    apt install --no-install-recommends -y \
    bash-completion \
    python3-pip \
    libboost-all-dev \
    software-properties-common \
    ros-foxy-rtabmap-ros \
    ros-foxy-pcl-ros \
    ros-foxy-perception-pcl

# Install gtsam
RUN add-apt-repository -y ppa:borglab/gtsam-release-4.1 && \
    apt install  --no-install-recommends -y libgtsam-dev libgtsam-unstable-dev

# Install pip dependencies
COPY requirements.txt /tmp/requirements.txt
RUN pip3 install -r /tmp/requirements.txt
RUN rm /tmp/requirements.txt

# There seems to be a version issue with numba
RUN pip3 install --upgrade numba

# Install TEASER++
RUN git clone https://github.com/MIT-SPARK/TEASER-plusplus.git && \
    cd TEASER-plusplus && \
    mkdir build && \
    cd build && \
    cmake .. && \
    make && \
    make install && \
    ldconfig && \
    cmake -DTEASERPP_PYTHON_VERSION=3.8 .. && \
    make teaserpp_python && \
    cd python && \
    pip3 install .

# Volume for repo
RUN mkdir -p /ros_ws/src

ENV USER=root

# Source ROS in .bashrc
RUN echo "alias sros1=\"source /opt/ros/noetic/setup.bash\"" >> /root/.bashrc
RUN echo "alias sros2=\"source /opt/ros/foxy/setup.bash\"" >> /root/.bashrc
RUN echo "alias sws=\"source /ros_ws/install/setup.bash\"" >> /root/.bashrc
RUN echo "sros2" >> /root/.bashrc
RUN echo "test -f /ros_ws/install/setup.bash && sws" >> /root/.bashrc

WORKDIR /ros_ws