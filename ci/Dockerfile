# Intended to build the antoinealb/cvra-ci docker image used for CI purpose.
# Automatically built from antoinealb's gitlab instance, see .gitlab-ci.yml
FROM ubuntu:18.04

RUN apt-get update && apt-get install -y \
        git \
        make \
        python3 \
        python3-pip \
        python3-dev \
        wget

RUN wget https://developer.arm.com/-/media/Files/downloads/gnu-rm/8-2018q4/gcc-arm-none-eabi-8-2018-q4-major-linux.tar.bz2 -O arm-gcc-linux.tar.bz2 && \
    tar -xf arm-gcc-linux.tar.bz2

RUN pip3 install cvra-packager~=1.0.0 \
    PyYAML==3.11 \
    msgpack-python~=0.5.6

ENV PATH="/gcc-arm-none-eabi-8-2018-q4-major/bin:${PATH}"
