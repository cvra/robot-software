# Actuator board firmware

https://github.com/cvra/actuator-board

## Loading via bootloader

```bash
make USE_BOOTLOADER=yes
bootloader_flash -b build/ch.bin -a 0x08003800 -c "actuator-board" 1
```
