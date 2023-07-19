ARG BASE_IMAGE=ubuntu:jammy
FROM ${BASE_IMAGE}

ENV DEBIAN_FRONTEND=noninteractive

# install dependencies
RUN apt-get update && apt-get install -y --no-install-recommends \
  && apt-get install -y --no-install-recommends wget nano build-essential cmake git clang lld \
  libpng-dev libjpeg-dev libeigen3-dev libboost-all-dev libglm-dev libglfw3-dev ca-certificates \
  libfmt-dev libspdlog-dev libassimp-dev zenity python3-pip \
  && apt-get clean \
  && rm -rf /var/lib/apt/lists/*

COPY . /root/iridescence
WORKDIR /root/iridescence
RUN rm -rf /root/iridescence/build
RUN git submodule init && git submodule update

# build
WORKDIR /root/iridescence/build
RUN cmake .. -DBUILD_EXAMPLES=ON -DBUILD_EXT_TESTS=ON
RUN make -j$(nproc)

WORKDIR /root/iridescence
RUN python3 setup.py build
RUN python3 setup.py install --user

ENV NVIDIA_VISIBLE_DEVICES ${NVIDIA_VISIBLE_DEVICES:-all}
ENV NVIDIA_DRIVER_CAPABILITIES ${NVIDIA_DRIVER_CAPABILITIES:+$NVIDIA_DRIVER_CAPABILITIES,}graphics

WORKDIR /

CMD ["bash"]
