#ifndef CRC16_H
#define CRC16_H

#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

// CRC-16-ANSI/IBM algorithm
uint16_t crc16(uint16_t init, const void *data, size_t length);

#ifdef __cplusplus
}
#endif

#endif /* CRC16_H */
