
#ifndef RPM_PORT_H
#define RPM_PORT_H

#ifdef __cplusplus
extern "C" {
#endif

#if defined(__unix__) || (defined(__APPLE__) && defined(__MACH__))

#define RPM_LOCK() {}

#define RPM_UNLOCK() {}

#else

#include <ch.h>

#define RPM_LOCK() {chSysLock();}

#define RPM_UNLOCK() {chSysUnlock();}

#endif

#ifdef __cplusplus
}
#endif

#endif /* RPM_PORT_H */

