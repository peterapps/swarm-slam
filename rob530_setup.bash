#!/bin/bash

alias sros1="source /opt/ros/noetic/setup.bash"
alias sros2="source /opt/ros/foxy/setup.bash"
alias sws="source ~/ros_ws/install/setup.bash"
alias cdws="cd ~/ros_ws"

DATA_DIR=~/ros_ws/src/swarm-slam/data/ros2_data 
DATA_INSTALL_DIR=~/ros_ws/install/cslam_experiments/share/cslam_experiments/data
alias sdata="ln -s $DATA_DIR $DATA_INSTALL_DIR"

buildws(){
    TMP_DIR=$(pwd)
    cdws
    sros2
    colcon build --symlink-install && sws && sdata
    cd $TMP_DIR
}

SHARED_DIR=/mnt/hgfs/project
REPO_DIR=~/ros_ws/src/swarm-slam

if [ ! -d $SHARED_DIR ]; then
    mntshared
fi

if [ ! -d $REPO_DIR ]; then
    ln -s $SHARED_DIR $REPO_DIR
fi

sros2
test -f /ros_ws/install/setup.bash && sws