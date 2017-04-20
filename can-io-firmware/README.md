# CAN IO firmware

## Loading via bootloader

```bash
make USE_BOOTLOADER=yes
bootloader_flash -b build/ch.bin -a 0x08003800 -c "can-io-board" 1
```
