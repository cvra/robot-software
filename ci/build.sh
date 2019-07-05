#!/usr/bin/env bash

# Make the script fail if any command in it fail
set -e

if [ "$BUILD_TYPE" == "" ]
then
    echo "Please define \$BUILD_TYPE!"
    exit 1
fi

source env/bin/activate
PROJPATH=$(pwd)

export PATH=$PROJPATH/gcc-arm-none-eabi-8-2018-q4-major/bin/:$PATH
export PATH=$PROJPATH/protoc/bin/:$PATH

export CFLAGS="$CFLAGS -I $HOME/cpputest/include/"
export CXXFLAGS="$CXXFLAGS -I $HOME/cpputest/include/"
export LDFLAGS="$CXXFLAGS -L $HOME/cpputest/lib/"

case $BUILD_TYPE in
    tests)
        mkdir build
        pushd build
        cmake ..
        make
        make test
        pop
        ;;

    build)
        mkdir build
        pushd build
        cmake .. -DCMAKE_TOOLCHAIN_FILE=../cmake/${PLATFORM}.cmake
        make ${PLATFORM}-firmware.elf
        popd
        ;;
    *)
        echo "Unknown build type $BUILD_TYPE"
        exit 1
        ;;
esac
