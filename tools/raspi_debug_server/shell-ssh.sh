#!/bin/bash
ssh-copy-id pi@192.168.1.202 && ssh pi@192.168.1.202 -t "python3 -m serial.tools.miniterm /dev/serial/by-id/usb-CVRA_ChibiOS_RT_Virtual_COM_Port_master-if00"