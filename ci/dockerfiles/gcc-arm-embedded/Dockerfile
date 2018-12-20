FROM ubuntu:trusty
MAINTAINER Club Vaudois de Robotique Autonome <info@cvra.ch>

# General dependencies
RUN apt-get update && apt-get install -y \
    git-core \
    make \
    python3-pip \
    python-software-properties \
    software-properties-common \
    unzip \
    wget \
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

RUN (cd /usr/local \
  && wget https://github.com/google/protobuf/releases/download/v3.5.1/protoc-3.5.1-linux-x86_64.zip \
  && unzip protoc*.zip \
  && chmod +x bin/protoc \
  && rm protoc*.zip \
)

RUN apt-get update && apt-get install -y \
    python-pip \
 && apt-get clean && rm -rf /var/lib/apt/lists/*
RUN pip install --upgrade protobuf
