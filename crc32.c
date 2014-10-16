#include <stdint.h>
#include <stdlib.h>
#include "crc32.h"

#define CRC32MASK 0xEDB88320

uint32_t crc32(uint32_t init, const uint8_t *data, size_t len)
{
    uint8_t *d = (uint8_t *)data;
    uint32_t i, crc = ~init;
    for (i = 0; i < len; i++) {
        uint32_t bit;
        for (bit = 0; bit < 8; bit++) {
            if ((crc & 1) != ((d[i]>>bit) & 1))
                crc = (crc >> 1) ^ CRC32MASK;
            else
                crc >>= 1;
        }
    }
    return ~crc;
}
