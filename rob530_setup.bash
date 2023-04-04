#!/bin/bash

alias sros1="source /opt/ros/noetic/setup.bash"
alias sros2="source /opt/ros/foxy/setup.bash"
alias sws="source ~/ros_ws/install/setup.bash"
alias cdws="cd ~/ros_ws"
alias mntshared="sudo mount -t fuse.vmhgfs-fuse .host:/ /mnt/hgfs -o allow_other"

buildws(){
    TMP_DIR=$(pwd)
    cdws
    sros2
    colcon build --symlink-install && sws
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