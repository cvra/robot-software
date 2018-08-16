#!/usr/bin/env bash

set -e
set -u

if [ $# -ne 2 ]
then
    echo "Usage $0 port file"
    exit 1
fi

PORT=$1
FILE=$2


BOARD_COUNT=`bootloader_read_config -a -p $PORT | grep "motor" | wc -l`

if [ $BOARD_COUNT -ne 13 ]
then
    echo "YO YOU ADD OR REMOVE BOARDS ON MY WATCH? YOU WANT A FIGHT?"
    exit
fi

echo "Found $BOARD_COUNT boards"

bootloader_flash -p $PORT \
    --base-address 0x08003800 \
    --binary $FILE \
    --device-class "motor-board-v1" \
    --run \
    20 21 22 23 25 27 28 29 31 32 44 67 68
