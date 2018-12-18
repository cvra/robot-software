FROM ubuntu:trusty
MAINTAINER Club Vaudois de Robotique Autonome <info@cvra.ch>

# General dependencies
RUN apt-get update && apt-get install -y \
    git-core \
    make \
    python3-pip \
    python-software-properties \
    software-properties-common \
 && apt-get clean && rm -rf /var/lib/apt/lists/*

RUN pip3 install \
    cvra-packager \
    msgpack-python==0.4.8 \
    PyYAML==3.11

# arm-none-eabi toolchain
RUN add-apt-repository ppa:team-gcc-arm-embedded/ppa && \
  apt-get update && apt-get install -y \
    gcc-arm-embedded \
  && apt-get clean && rm -rf /var/lib/apt/lists/*

