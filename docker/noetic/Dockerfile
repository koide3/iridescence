FROM ubuntu:focal

ENV DEBIAN_FRONTEND=noninteractive

# install dependencies
RUN apt-get update && apt-get install -y --no-install-recommends \
  && apt-get install -y --no-install-recommends wget nano build-essential cmake git clang lld \
  libpng-dev libeigen3-dev libboost-all-dev libglm-dev libglfw3-dev ca-certificates \
  && apt-get clean \
  && rm -rf /var/lib/apt/lists/*

COPY . /root/iridescence
WORKDIR /root/iridescence
RUN rm -rf /root/iridescence/build
RUN git submodule init && git submodule update

# install ROS
RUN apt-get update \
  && apt-get install -y --no-install-recommends \
  && apt-get install -y --no-install-recommends gnupg gnupg2 \
  && apt-get clean && rm -rf /var/lib/apt/lists/*

RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu focal main" > /etc/apt/sources.list.d/ros-latest.list'
RUN apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654

# install dependencies
RUN apt-get update \
  && apt-get install -y --no-install-recommends \
  && apt-get install -y --no-install-recommends ros-noetic-pcl-ros \
  && apt-get clean && rm -rf /var/lib/apt/lists/*

RUN mkdir -p /root/catkin_ws/src
WORKDIR /root/catkin_ws/src
RUN /bin/bash -c '. /opt/ros/noetic/setup.bash; catkin_init_workspace'

RUN mv /root/iridescence /root/catkin_ws/src/iridescence

WORKDIR /root/catkin_ws
RUN /bin/bash -c '. /opt/ros/noetic/setup.bash; catkin_make -DCMAKE_BUILD_TYPE=Release'

# RUN sed -i "6i source \"/root/catkin_ws/devel/setup.bash\"" /ros_entrypoint.sh

WORKDIR /

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
