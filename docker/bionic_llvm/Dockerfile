FROM ubuntu:bionic

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y --no-install-recommends \
  && apt-get install -y --no-install-recommends wget nano build-essential cmake git clang lld \
  libeigen3-dev libboost-all-dev libglm-dev libglfw3-dev \
  && apt-get clean \
  && rm -rf /var/lib/apt/lists/*

COPY . /root/iridescent
WORKDIR /root/iridescent
RUN rm -rf /root/iridescent/build
RUN git submodule init && git submodule update

WORKDIR /root/iridescent/build
RUN cmake .. && make -j$(nproc)

ENV NVIDIA_VISIBLE_DEVICES ${NVIDIA_VISIBLE_DEVICES:-all}
ENV NVIDIA_DRIVER_CAPABILITIES ${NVIDIA_DRIVER_CAPABILITIES:+$NVIDIA_DRIVER_CAPABILITIES,}graphics

WORKDIR /

CMD ["bash"]
