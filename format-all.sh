#!/usr/bin/env bash

set -eu

cd $(dirname $0)

SRC=$(git ls-tree --full-tree -r HEAD | grep -e "\.\(c\|h\|hpp\|cpp\|proto\)\$" | cut -f 2)

clang-format -style=file -i $SRC
