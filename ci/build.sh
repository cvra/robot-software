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
export PATH=$PROJPATH/gcc-arm-none-eabi-4_9-2014q4/bin/:$PATH

export CFLAGS="$CFLAGS -I $HOME/cpputest/include/"
export CXXFLAGS="$CXXFLAGS -I $HOME/cpputest/include/"
export LDFLAGS="$CXXFLAGS -L $HOME/cpputest/lib/"

# Generate base nanopb messages
pushd lib/nanopb/generator/proto
make
popd

pushd master-firmware

packager

case $BUILD_TYPE in
    tests)
        mkdir build
        cd build
        cmake ..
        make check
        ;;

    build)
        make dsdlc
        make
        ;;
    *)
        echo "Unknown build type $BUILD_TYPE"
        exit 1
        ;;
esac

popd
