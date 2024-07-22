#!/usr/bin/env bash


TOP_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd -P)"


XAUTH=$HOME/.Xauthority
USER_ID="$(id -u)"
SHARED_DOCKER_DIR="/home/autoware"

XSOCK=/tmp/.X11-unix
XAUTH_DOCKER=$HOME/.Xauthority

VOLUMES="--volume=$XSOCK:$XSOCK:rw
--volume=$XAUTH:$XAUTH_DOCKER:rw"

AUTOWARE_HOST_DIR=$TOP_DIR
AUTOWARE_DOCKER_DIR=/home/autoware

VOLUMES="$VOLUMES --volume=$AUTOWARE_HOST_DIR:$AUTOWARE_DOCKER_DIR "

CONTAINER_NAME="witpilot"
IMAGE_NAME_DEV="autoware/autoware:bleedingedge-melodic-base-cuda"

T="-t" # ci job can not use -it option
RM="--rm"
RUNTIME="--gpus all"
PRIVILEGED=" --privileged "
PUBLISH="  --net=host "
docker run \
  -i $T $RM --name $CONTAINER_NAME $VOLUMES \
  --env="XAUTHORITY=${XAUTH}" \
  --env="DISPLAY=${DISPLAY}" \
  --env="USER_ID=$USER_ID" \
  --shm-size 8G \
  --env="QT_X11_NO_MITSHM=1" \
  --workdir $SHARED_DOCKER_DIR $PRIVILEGED $PUBLISH \
  $RUNTIME \
  ${IMAGE_NAME_DEV}

# docker exec -it -w /home/autoware -u 1000 witpilot bash