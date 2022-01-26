#include <string.h>
#include "bootloader_config.h"

bool config_get(bootloader_config_t* cfg)
{
#if !defined(CONFIG_ADDR)
    // not compiled for bootloader, use default config
    const char* default_id = "dummy_config";
    const char* device_class = "motor-board-v1";
    cfg->ID = 1;
    strcpy(cfg->board_name, default_id);
    strcpy(cfg->device_class, device_class);
    cfg->application_crc = 0;
    cfg->application_size = 0;
    cfg->update_count = 0;
    return true;
#else
    uint8_t* p = (uint8_t*)CONFIG_ADDR;
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
#endif
}
