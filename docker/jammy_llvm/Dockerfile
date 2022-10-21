FROM ubuntu:jammy

ENV DEBIAN_FRONTEND=noninteractive

# install dependencies
RUN apt-get update && apt-get install -y --no-install-recommends \
  && apt-get install -y --no-install-recommends wget nano build-essential cmake git clang lld \
  libpng-dev libjpeg-dev libeigen3-dev libboost-all-dev libglm-dev libglfw3-dev ca-certificates \
  && apt-get clean \
  && rm -rf /var/lib/apt/lists/*

COPY . /root/iridescence
WORKDIR /root/iridescence
RUN rm -rf /root/iridescence/build
RUN git submodule init && git submodule update

# build
RUN update-alternatives --install /usr/bin/ld ld /usr/bin/ld.lld 50

WORKDIR /root/iridescence/build
RUN CC=clang CXX=clang++ cmake .. && make -j$(nproc)

ENV NVIDIA_VISIBLE_DEVICES ${NVIDIA_VISIBLE_DEVICES:-all}
ENV NVIDIA_DRIVER_CAPABILITIES ${NVIDIA_DRIVER_CAPABILITIES:+$NVIDIA_DRIVER_CAPABILITIES,}graphics

WORKDIR /

CMD ["bash"]
