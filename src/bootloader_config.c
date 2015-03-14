#include "bootloader_config.h"

bool config_get(bootloader_config_t *cfg)
{
#if !defined(CONFIG_ADDR)
#error "not compiled for bootloader"
#endif
    uint8_t *p = (uint8_t *) CONFIG_ADDR;
    // first config page
    if (config_is_valid(p, CONFIG_PAGE_SIZE)) {
        *cfg = config_read(p, CONFIG_PAGE_SIZE);
        return true;
    }
    // try second config page
    p += CONFIG_PAGE_SIZE;
    if (config_is_valid(p, CONFIG_PAGE_SIZE)) {
        *cfg = config_read(p, CONFIG_PAGE_SIZE);
        return true;
    }
    return false;
}
