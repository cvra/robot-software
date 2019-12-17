#ifndef VERSION_H
#define VERSION_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

extern const char* software_version_str;
extern const char* hardware_version_str;
extern const uint32_t software_version_short;

#ifdef __cplusplus
}
#endif

#endif /* VERSION_H */
