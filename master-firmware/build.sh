#!/bin/sh

# red colored summary of submodules that are not updated
printf "\033[1;31m\n"
git submodule summary | sed -e "s/\x1b\[.\{1,5\}m//g" # the regex removes color
printf "\033[0m"

printf "\033[1;34m\n%s\033[0m\n" "Running packager"
packager

printf "\033[1;34m\n%s\033[0m\n" "generate MessagePack config"
make config_msgpack

make dsdlc

make
