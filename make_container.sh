#!/bin/bash

TAG_NAME=swarmslam_test2
DOCKERFILE_NAME=Dockerfile

SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )

docker build ${SCRIPT_DIR} -t ${TAG_NAME} #-f ${DOCKERFILE_NAME}
docker create \
    --name rob530_test2 \
    --privileged \
    --volume ${SCRIPT_DIR}:/ros_ws/src/swarmslam \
    --interactive \
    --env DISPLAY=host.docker.internal:0 \
    --volume /tmp/.X11-unix:/tmp/.X11-unix \
    --network=host \
    ${TAG_NAME}:latest
    # "--privileged" from https://answers.ros.org/question/301056/ros2-rviz-in-docker-container/?answer=301062#post-id-301062

echo 'Done'