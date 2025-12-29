ARG BASE_IMAGE=ubuntu:jammy
FROM ${BASE_IMAGE}

ENV DEBIAN_FRONTEND=noninteractive

# install dependencies
RUN apt-get update && apt-get install -y --no-install-recommends \
  && apt-get install -y --no-install-recommends wget nano build-essential cmake git clang lld ninja-build \
  libpng-dev libjpeg-dev libeigen3-dev libglm-dev libglfw3-dev libgtest-dev ca-certificates \
  libfmt-dev libspdlog-dev libassimp-dev zenity python3-dev python3-pip python3-pybind11 \
  && apt-get clean \
  && rm -rf /var/lib/apt/lists/*

RUN mkdir -p ~/.config/pip
RUN echo "[global]\nbreak-system-packages=true" > ~/.config/pip/pip.conf

COPY . /root/iridescence
WORKDIR /root/iridescence
RUN rm -rf /root/iridescence/build
RUN git submodule init && git submodule update

# build
RUN update-alternatives --install /usr/bin/ld ld /usr/bin/ld.lld 50

WORKDIR /root/iridescence/build
RUN CC=clang CXX=clang++ cmake .. -DBUILD_EXAMPLES=ON -DBUILD_EXT_TESTS=ON
RUN make -j$(nproc)

WORKDIR /root/iridescence
RUN pip install . --user

ENV NVIDIA_VISIBLE_DEVICES ${NVIDIA_VISIBLE_DEVICES:-all}
ENV NVIDIA_DRIVER_CAPABILITIES ${NVIDIA_DRIVER_CAPABILITIES:+$NVIDIA_DRIVER_CAPABILITIES,}graphics

WORKDIR /

CMD ["bash"]
