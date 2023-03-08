FROM ros:foxy

LABEL Name=swarmslam Version=0.0.1

# Arguments
ARG DEBIAN_FRONTEND=noninteractive
ARG USER_NAME=rob
ARG USER_UID=1000
ARG USER_GID=1000

# Install APT dependencies
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

# Fix issue with the version of numba in requirements.txt
RUN pip3 install --upgrade numba

# Create user
ENV USER=${USER_NAME}
RUN groupadd ${USER_NAME} --gid ${USER_GID}\
    && useradd -l -m ${USER_NAME} -u ${USER_UID} -g ${USER_GID} -s /bin/bash
ENV DEBIAN_FRONTEND=noninteractive
ENV USER=${USER_NAME}
RUN echo "rob ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/${USER_NAME}
RUN chmod 0440 /etc/sudoers.d/${USER_NAME}

# Volume for repo
RUN mkdir -p /ros_ws/src
RUN chown -R ${USER_NAME}:${USER_NAME} /ros_ws

# Source ROS in .bashrc
USER ${USER_NAME}
RUN echo "source /ros_entrypoint.sh" >> /home/${USER_NAME}/.bashrc
RUN echo "test -f /ros_ws/install/setup.bash && source /ros_ws/install/setup.bash" >> /home/${USER_NAME}/.bashrc

# Use bash instead of sh as a shell
RUN echo "dash dash/sh boolean false" | sudo debconf-set-selections
RUN sudo DEBIAN_FRONTEND=noninteractive dpkg-reconfigure dash
ENV ENV ~/.profile

WORKDIR /ros_ws
SHELL ["/bin/bash", "-c"]
CMD ["/bin/bash"]