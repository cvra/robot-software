#ifndef CC_H
#define CC_H

#include <stdint.h>
#include <ch.h>

/* Define generic types used in lwIP */
typedef uint8_t u8_t;
typedef int8_t s8_t;
typedef uint16_t u16_t;
typedef int16_t s16_t;
typedef uint32_t u32_t;
typedef int32_t s32_t;
typedef intptr_t mem_ptr_t;/* Define generic types used in lwIP */
typedef uint8_t u8_t;
typedef int8_t s8_t;
typedef uint16_t u16_t;
typedef int16_t s16_t;
typedef uint32_t u32_t;
typedef int32_t s32_t;
typedef intptr_t mem_ptr_t;

#define BYTE_ORDER LITTLE_ENDIAN


/* Diagnostic macros. */
#define LWIP_PLATFORM_ASSERT(msg) chSysHalt(msg)


#endif
