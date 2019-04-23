#include <string.h>
#include "uid.h"

#define UID_REGISTER ((const uint8_t*)(0x1FFF7A10))

void uid_read(uint8_t out[UID_LENGTH])
{
    memcpy(out, UID_REGISTER, UID_LENGTH);
}
