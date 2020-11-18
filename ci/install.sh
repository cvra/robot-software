#!/usr/bin/env bash

# Make the script fail if any command in it fail
set -e

python3 -m venv env --without-pip
source env/bin/activate
python --version
wget https://bootstrap.pypa.io/get-pip.py
python get-pip.py
pip install cvra-packager~=1.0.0
pip install msgpack-python==0.4.8 PyYAML==3.11
pip install --upgrade protobuf

pushd lib/uavcan/libuavcan/dsdl_compiler/pyuavcan/
python setup.py install
popd

pushd lib/uavcan/libuavcan/dsdl_compiler/
python setup.py install
popd

if [ "$BUILD_TYPE" != "raspberry" ]
then
    pushd lib/nanopb/nanopb/generator/proto
    make
    popd
fi

if [ "$BUILD_TYPE" == "build-cmake" -o "$BUILD_TYPE" == "build-packager" ]
then
    wget https://developer.arm.com/-/media/Files/downloads/gnu-rm/9-2020q2/gcc-arm-none-eabi-9-2020-q2-update-x86_64-linux.tar.bz2 -O arm-gcc-linux.tar.bz2
    tar -xf arm-gcc-linux.tar.bz2
fi

if [ "$BUILD_TYPE" == "raspberry" ]
then
    docker pull antoinealb/cvra-sdk
fi

if [ "$BUILD_TYPE" == "tests" ]
then
    # Install cpputest
    pushd ..
    wget "https://github.com/cpputest/cpputest/releases/download/v3.8/cpputest-3.8.tar.gz" -O cpputest.tar.gz
    tar -xzf cpputest.tar.gz
    cd cpputest-3.8/
    ./configure --prefix=$HOME/cpputest
    make
    make install
    popd
fi
