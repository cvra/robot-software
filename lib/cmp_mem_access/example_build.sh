#!/bin/sh

git clone https://github.com/camgunz/cmp.git
clang cmp_mem_access.c cmp/cmp.c example.c -I . -o example
./example
