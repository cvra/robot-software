#ifndef UID_H
#define UID_H

#include <stdint.h>

#define UID_LENGTH 12

/** Returns the microcontroller's unique ID */
void uid_read(uint8_t out[UID_LENGTH]);

#endif
