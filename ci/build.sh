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

export PATH=$PROJPATH/gcc-arm-none-eabi-9-2020-q2-update/bin/:$PATH
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
        popd
        ;;

    raspberry)
        docker run -it \
            --volume $(pwd):/src \
            -w /src/lib/nanopb/nanopb/generator/proto antoinealb/cvra-sdk make
        mkdir build
        docker run -it -v $(pwd):/src \
            -w /src/build-docker antoinealb/cvra-sdk \
            cmake .. -DCMAKE_TOOLCHAIN_FILE=/aarch64-buildroot-linux-gnu_sdk-buildroot/share/buildroot/toolchainfile.cmake
        docker run -it -v $(pwd):/src \
        -w /src/build-docker antoinealb/cvra-sdk \
        make master-firmware.ipk
        ;;

    build-cmake)
        mkdir build
        pushd build
        cmake .. -DCMAKE_TOOLCHAIN_FILE=../cmake/${PLATFORM}.cmake
        make ${PLATFORM}-firmware.elf
        popd
        ;;

    build-packager)
        pushd ${PLATFORM}-firmware
        packager
        make dsdlc
        make
        popd
        ;;

    *)
        echo "Unknown build type $BUILD_TYPE"
        exit 1
        ;;
esac
