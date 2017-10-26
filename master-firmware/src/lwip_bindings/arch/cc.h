#ifndef CC_H
#define CC_H

#include <stdint.h>
#include <ch.h>

#include <error/error.h>

#define U16_F "hu"
#define S16_F "hd"
#define X16_F "hx"
#define U32_F "u"
#define S32_F "d"
#define X32_F "x"

/* Diagnostic macros. */
#define LWIP_PLATFORM_ASSERT(msg) ERROR(msg)
#define LWIP_PLATFORM_DIAG(msg) do {DEBUG msg;} while (0);

/* Tell lwIP to use its own defines() for standard unix error codes */
#define LWIP_PROVIDE_ERRNO


#endif
