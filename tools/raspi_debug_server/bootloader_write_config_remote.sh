#!/bin/bash
if ssh-keygen -F 192.168.1.201 >/dev/null;
then echo SSH key found;
else ssh-copy-id pi@192.168.1.201 >/dev/null
fi
ssh pi@192.168.1.201 -t /home/pi/.local/bin/bootloader_write_config $@