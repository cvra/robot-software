#ifndef CRC32_H
#define CRC32_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdlib.h>

uint32_t crc32(uint32_t init, const void *data, size_t len);

#ifdef __cplusplus
}
#endif

#endif
