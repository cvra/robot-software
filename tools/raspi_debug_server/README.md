# Raspberry as server to debug/flash

Features :
 * Firmware upload
 * Acces the USB shell                                          
 * Connect to the CAN traffic 

Raspberry:
 * port 3333 : gdb
 * port 4444 : telnet

 To do:
 * build a small thing for can-bus (so it doesn't break the chain)
 * make services work for openocd
 * launch the ./shell-shh.sh script via a cvra cmd
 * modify the script so we can launch other commande too (bootloader_*)


# Command (from your computer) 
Flash remotly
```BASH
make flash-pi
```
Get chibios shell
```BASH
./shell-shh.sh
```

# Setup rasppi
## Update and upgrade the pi
```BASH
sudo apt update -y && sudo apt upgrade -y
```
## Install Python3 and Co
```BASH
sudo apt install python3
```
```BASH
sudo apt install python3-pip && pip3 install cvra-bootloader
```
```BASH
pip3 install pyserial
```
Update $PATH to find python
```BASH
vim ~/.bashrc 
```
by adding this at the eof
```BASH
export PATH=$PATH:~/.local/bin/
```

## Setup a service (with systemd) to launch our openocd at boot
```BASH
sudo mv openocd-remote.service /lib/systemd/system/.
```
```BASH
sudo systemctl daemon-reload
```
```BASH
sudo systemctl enable openocd-remote.service
```
Check if it is enabled:
```BASH
sudo systemctl list-unit-files | grep enabled
```
then
```BASH
sudo systemctl start openocd-remote.service
```
or 
```BASH
reboot
```


# note:
https://www.st.com/content/ccc/resource/technical/document/user_manual/group0/26/49/90/2e/33/0d/4a/da/DM00244518/files/DM00244518.pdf/jcr:content/translations/en.DM00244518.pdf
p.19 -> gpio for flash



/////
gdb port 3333

Lire transfert
lire .elf

sudo openocd -f oocd_raspberry.cfg -c "program ch.elf verify reset" -c "shutdown"
gdb:
    arm-none-eabi-gdb build/ch.elf -ex "target remote 192.168.43.96:3333" -ex "load" -ex "detach -ex "quit"

ssh pi@192.168.43.96 -l pi -t "python3 -m serial.tools.miniterm /dev/serial/by-id/usb-CVRA_ChibiOS_RT_Virtual_COM_Port_master-if00" 



systemd services -> sur le raspi pour lancer openocd au démarage
https://www.tanzolab.it/systemd
___
___
___
___
![hey](https://i.imgflip.com/2uhb5u.jpg)
