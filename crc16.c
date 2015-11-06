#include <stdint.h>
#include <stddef.h>
#include "crc16.h"

#define CRC16_POLYNOMIAL    0xA001 // bit reversed 0x8005

uint16_t crc16(uint16_t init, const void *data, size_t length)
{
    uint8_t *p = (uint8_t *)data;
    uint16_t crc = init;
    while (length--) {
        int i;
        crc ^= *p++;
        for (i = 0; i < 8; i++) {
            if (crc & 1) {
                crc = (crc >> 1) ^ CRC16_POLYNOMIAL;
            } else {
                crc = crc >> 1;
            }
        }
    }
    return crc;
}
