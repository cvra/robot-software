#include <stdint.h>
#include <stdlib.h>
#include "crc32.h"

#define CRC32_POLYNOMIAL    0xEDB88320 // bit reversed 0x04C11DB7

uint32_t crc32(uint32_t init, const void *data, size_t length)
{
    uint8_t *p = (uint8_t *)data;
    uint32_t crc = ~init;
    size_t i;
    for (i = 0; i < length; i++) {
        int bit;
        for (bit = 0; bit < 8; bit++) {
            if ((crc & 1) != ((p[i]>>bit) & 1)) {
                crc = (crc >> 1) ^ CRC32_POLYNOMIAL;
            } else {
                crc = crc >> 1;
            }
        }
    }
    return ~crc;
}
