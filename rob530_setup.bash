#!/bin/bash

WS_DIR=~/ros_ws
SHARED_DIR=/mnt/hgfs/project
REPO_DIR=${WS_DIR}/src/swarm-slam
DATA_DIR=${REPO_DIR}/data/ros2_data 
DATA_INSTALL_DIR=${WS_DIR}/install/cslam_experiments/share/cslam_experiments/data

alias sbashrc="source ~/.bashrc"
alias sros1="source /opt/ros/noetic/setup.bash"
alias sros2="source /opt/ros/foxy/setup.bash"
alias sws="source ${WS_DIR}/install/setup.bash"
alias cdws="cd ${WS_DIR}"
alias cdrepo="cd ${REPO_DIR}"
alias sdata="ln -s ${DATA_DIR} ${DATA_INSTALL_DIR}"

if [ ! -d $REPO_DIR ]; then
    ln -s $SHARED_DIR $REPO_DIR
fi

pyclean(){
    find . -regex '^.*\(__pycache__\|\.py[co]\)$' -delete
}

buildws(){
    TMP_DIR=$(pwd)
    cdws
    sros2
    colcon build --symlink-install && sws && sdata
    cd $TMP_DIR
}

cleanws(){
    TMP_DIR=$(pwd)
    cdws
    rm -rf ./build ./install ./log
    pyclean
    cdrepo
    cd $TMP_DIR
}

testws(){
    TMP_DIR=$(pwd)
    cdws
    colcon test
    colcon test-result --verbose
    cd $TMP_DIR
}

# Alternative to modifying fstab
# https://kb.vmware.com/s/article/60262
# alias mntshared="sudo mount -t fuse.vmhgfs-fuse .host:/ /mnt/hgfs -o allow_other"
# alias mntshared="sudo mount -t fuse.vmhgfs-fuse .host:/ /mnt -o allow_other"
# if [ ! -d $REPO_DIR ]; then
#     ln -s $SHARED_DIR $REPO_DIR
# fi

sros2
test -f ${WS_DIR}/install/setup.bash && sws