FROM ubuntu:bionic

ENV DEBIAN_FRONTEND=noninteractive

# install dependencies
RUN apt-get update && apt-get install -y --no-install-recommends \
  && apt-get install -y --no-install-recommends wget nano build-essential cmake git clang lld \
  libpng-dev libeigen3-dev libboost-all-dev libglm-dev libglfw3-dev \
  && apt-get clean \
  && rm -rf /var/lib/apt/lists/*

COPY . /root/iridescence
WORKDIR /root/iridescence
RUN rm -rf /root/iridescence/build
RUN git submodule init && git submodule update

