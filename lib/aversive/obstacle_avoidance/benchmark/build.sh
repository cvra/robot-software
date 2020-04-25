#!/bin/sh
CC=clang++
CFLAGS=-I../../../../lib

cd $(dirname $0)

$CC $CFLAGS -o benchmark -lbenchmark -O3 \
    main.cpp \
    ../../math/geometry/polygon.c \
    ../../math/geometry/lines.c \
    ../../math/geometry/vect_base.c \
    ../obstacle_avoidance.c
