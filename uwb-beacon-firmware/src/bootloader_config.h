#ifndef BOOTLOADER_CONFIG_H
#define BOOTLOADER_CONFIG_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>
#include <can-bootloader/config.h>

// returns true if config read was successful
bool config_get(bootloader_config_t* cfg);

#ifdef __cplusplus
}
#endif

#endif /* BOOTLOADER_CONFIG_H */
