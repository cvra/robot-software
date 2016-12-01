FROM ubuntu:trusty
MAINTAINER Club Vaudois de Robotique Autonome <info@cvra.ch>

RUN apt-get update && apt-get install -y \
    autoconf \
    automake \
    cmake \
    build-essential \
    git-core \
    libtool \
    python3-pip

RUN git clone https://github.com/cpputest/cpputest && \
    cd cpputest && \
    autoreconf . -i && \
    ./configure && \
    make && make install

RUN pip3 install cvra-packager

RUN apt-get autoremove -y autoconf \
    automake \
    libtool

RUN apt-get clean && rm -rf /var/lib/apt/lists/* /tmp/* /var/tmp/*
