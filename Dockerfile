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
    ros-foxy-perception-pcl \
    ros-foxy-rviz2

    # apt-get install -y libgl1-mesa-dev freeglut3-dev mesa-common-dev
    # apt-get install -y mesa-utils libgl1-mesa-glx

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
RUN chown -R ${USER_NAME}:${USER_NAME} /ros_ws

# Source ROS in .bashrc
USER ${USER_NAME}
RUN echo "source /ros_entrypoint.sh" >> /home/${USER_NAME}/.bashrc
RUN echo "test -f /ros_ws/install/setup.bash && source /ros_ws/install/setup.bash" >> /home/${USER_NAME}/.bashrc

# Add display handling stuff
RUN echo 'export LIBGL_ALWAYS_INDIRECT=1' >> home/${USER_NAME}/.bashrc
RUN echo 'export MESA_GL_VERSION_OVERRIDE=3.3' >> home/${USER_NAME}/.bashrc

# Use bash instead of sh as a shell
RUN echo "dash dash/sh boolean false" | sudo debconf-set-selections
RUN sudo DEBIAN_FRONTEND=noninteractive dpkg-reconfigure dash
ENV ENV ~/.profile

WORKDIR /ros_ws