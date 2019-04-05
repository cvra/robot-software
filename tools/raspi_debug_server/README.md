# Raspberry as server to debug/flash

Features :
 * Firmware upload
 * Access the USB shell
 * Connect to the CAN traffic 

Raspberry:
 * port 3333 : gdb
 * port 4444 : telnet

 To do:
 * launch the ./shell-shh.sh script via a cvra cmd
 * modify the script so we can launch other commande too (bootloader_*)
 * faster way to flash
 * remove the power of the emergency stop line
 * a case


# Command (from your computer) 
Flash remotly
```BASH
make flash-pi
```
Get chibios shell
```BASH
./shell-ssh.sh
```
Copy your ssh id for autologing
```BASH
ssh-copy-id pi@192.168.1.201
```

# Setup rasppi
## Update and upgrade the pi
```BASH
sudo apt update -y && sudo apt upgrade -y
```
## Install Python3 and Co
```BASH
sudo apt install python3
sudo apt install python3-pip
pip3 install --user cvra-bootloader pyserial
# update $PATH to find python
echo "export PATH=$PATH:~/.local/bin/" >> ~/.bashrc
```


## Setup a service (with systemd) to launch our openocd at boot
```BASH
sudo cp openocd-remote.service /lib/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable openocd-remote.service

# check if it is enabled:
systemctl status openocd-remote.service

# start and reboot
sudo systemctl start openocd-remote.service
reboot
```