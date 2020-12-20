FROM ubuntu:focal

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y --no-install-recommends \
  && apt-get install -y --no-install-recommends wget nano build-essential cmake git \
  libeigen3-dev libboost-dev libboost-filesystem-dev libboost-program-options-dev \
  libpng-dev libglm-dev libglfw3-dev \
  && apt-get clean \
  && rm -rf /var/lib/apt/lists/*

COPY . /root/iridescence
WORKDIR /root/iridescence
RUN rm -rf /root/iridescence/build
RUN git submodule init && git submodule update

WORKDIR /root/iridescence/build
RUN cmake .. && make -j$(nproc)

ENV NVIDIA_VISIBLE_DEVICES ${NVIDIA_VISIBLE_DEVICES:-all}
ENV NVIDIA_DRIVER_CAPABILITIES ${NVIDIA_DRIVER_CAPABILITIES:+$NVIDIA_DRIVER_CAPABILITIES,}graphics

WORKDIR /

CMD ["bash"]
