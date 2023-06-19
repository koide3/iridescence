#!/bin/bash
xhost + local:root

sudo docker run -it --rm \
			--gpus all \
			-e DISPLAY=$DISPLAY \
			-v /tmp/.X11-unix:/tmp/.X11-unix \
                        -v /dev:/dev \
			$@

# xhost - local:root
