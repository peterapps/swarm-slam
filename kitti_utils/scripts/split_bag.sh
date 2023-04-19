#!/bin/bash

if [ $# -eq 0 ]
  then
    echo "Usage: split_bag.sh ROS1_BAG_FILE.bag"
    exit
fi

source /opt/ros/noetic/setup.bash

BAG_FILE=$1
EXTENSION="${BAG_FILE##*.}"
FILE_NAME="${BAG_FILE%.*}"

OUT1=${FILE_NAME}-part1.${EXTENSION}
OUT2=${FILE_NAME}-part2.${EXTENSION}

START_TIME=$(rosbag info -y -k start ${BAG_FILE})
END_TIME=$(rosbag info -y -k start ${BAG_FILE})
MID_TIME=`echo "$START_TIME + ($END_TIME - $START_TIME) * 0.5" | bc -l`

echo "Writing first half to ${OUT1}"
rosbag filter ${BAG_FILE} ${OUT1} "t.secs <= ${MID_TIME}"
echo "Writing second half to ${OUT2}"
rosbag filter ${BAG_FILE} ${OUT2} "t.secs > ${MID_TIME}"