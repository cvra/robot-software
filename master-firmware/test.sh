#!/usr/bin/env bash

printf "\033[1;34m\n%s\033[0m\n" "Running packager"
packager

mkdir -p build
cd build
cmake ..
make check
