# Raspberry as server to debug/flash

Features :
 * Firmware upload
 * (done) Acces the USB shell                                          
 * (done) Connect to the CAN traffic                                   
 * (try with gpio first) the STLink for firmware upload                               
 * (done) power the raspi through the CAN bus                          
 * use openocd GPIO adapter instead of st link, might work better?

 To do:
 * find how to flash via pi gpio
 * Try to scp build and flash it
 * build a small thing for can-bus (so it doesn't break the chain)


# Setup rasppi
### Update and upgrade the pi
```BASH
sudo apt update -y && sudo apt upgrade -y
```
### Install Python3 and Co
```BASH
sudo apt install python3
```
```BASH
sudo apt install python3-pip && pip3 install cvra-bootloader
```
```BASH
pip3 install pyserial
```
### Update $PATH to find python
```BASH
vim ~/.bashrc 
```
### and add at the end of the file
```BASH
export PATH=$PATH:~/.local/bin/
```
```BASH
sudo apt install openocd
```
Check if I really had to install libqueue

# On your computer 
To save your public to the raspberry.
```BASH
ssh-copy-id <USERNAME>@<IP-ADDRESS>
```
```BASH
scp fun pi@192.168.43.229:test/. && ssh pi@192.168.43.229 "mv test/* ."
```
___
___
![hey](https://i.imgflip.com/2uhb5u.jpg)

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