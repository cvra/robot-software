#ifndef CRC32_H
#define CRC32_H

#include <stdint.h>
#include <stdlib.h>

#ifdef __cplusplus
extern "C" {
#endif

// Standard Ethernet CRC-32 algoritm using polynomial 0x04C11DB7
uint32_t crc32(uint32_t init, const void *data, size_t length);

#ifdef __cplusplus
}
#endif

#endif
