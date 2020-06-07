git submodule update --init --recursive
docker run -it -v $(pwd):/src -w /src/lib/nanopb/nanopb/generator/proto cvra-buildroot make
mkdir build-docker
docker run -it -v $(pwd):/src -w /src/build-docker cvra-buildroot cmake .. -DCMAKE_TOOLCHAIN_FILE=/aarch64-buildroot-linux-gnu_sdk-buildroot/share/buildroot/toolchainfile.cmake
docker run -it -v $(pwd):/src -w /src/build-docker cvra-buildroot make master-firmware.ipk
