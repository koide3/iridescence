#!/bin/bash
iname=${DOCKER_IMAGE:-"iridescence"} ## name of image
cname=${DOCKER_CONTAINER:-"iridescence"} ## name of container

SUDO_STRING=`groups|grep docker`
SUDO=""
if [ -z "$SUDO_STRING" ]; then
  SUDO="sudo "
fi

VAR="${@:-"bash"}"
if [ $# -eq 0 -a -z "$OPT" ]; then
    OPT=-it
fi

if [ -z "$NO_GPU" ]; then
    GPU_OPT='--gpus all,"capabilities=compute,graphics,utility,display"'
else
    GPU_OPT=""
fi

NET_OPT="--net=host"

if [ -n "$USE_USER" ]; then
    USER_SETTING=" -u $(id -u):$(id -g) -v /etc/passwd:/etc/passwd:ro -v /etc/group:/etc/group:ro"
fi

xhost +si:localuser:root
rmimage=$(docker rm ${cname})

$SUDO docker run \
    --privileged     \
    ${OPT}           \
    ${GPU_OPT}       \
    ${NET_OPT}       \
    ${USER_SETTING}  \
    --env="DISPLAY"  \
    --env="QT_X11_NO_MITSHM=1" \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
	--volume="/dev:/dev" \
	--name=${cname} \
    ${iname} \
    ${VAR}


## capabilities
# compute	CUDA / OpenCL アプリケーション
# compat32	32 ビットアプリケーション
# graphics	OpenGL / Vulkan アプリケーション
# utility	nvidia-smi コマンドおよび NVML
# video		Video Codec SDK
# display	X11 ディスプレイに出力
# all