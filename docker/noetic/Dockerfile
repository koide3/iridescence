FROM ros:noetic

RUN apt-get update \
  && apt-get install -y --no-install-recommends \
  && apt-get install -y --no-install-recommends wget nano git libomp-dev build-essential python3-pip \
  libeigen3-dev libboost-all-dev libglm-dev libglfw3-dev \
  ros-noetic-pcl-ros \
  && apt-get clean && rm -rf /var/lib/apt/lists/*

RUN mkdir -p /root/catkin_ws/src
WORKDIR /root/catkin_ws/src
RUN /bin/bash -c '. /opt/ros/noetic/setup.bash; catkin_init_workspace'

COPY . /root/catkin_ws/src/iridescent/
WORKDIR /root/catkin_ws/src/iridescent
RUN git submodule init && git submodule update

WORKDIR /root/catkin_ws
RUN /bin/bash -c '. /opt/ros/noetic/setup.bash; catkin_make -DCMAKE_BUILD_TYPE=Release'

RUN sed -i "6i source \"/root/catkin_ws/devel/setup.bash\"" /ros_entrypoint.sh

WORKDIR /

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
