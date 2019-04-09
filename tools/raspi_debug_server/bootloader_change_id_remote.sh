#!/bin/bash
set -eu -o pipefail
ssh pi@192.168.1.201 -t /home/pi/.local/bin/bootloader_change_id $@