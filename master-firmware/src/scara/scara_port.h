#ifndef SCARA_PORT_H
#define SCARA_PORT_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* returns time since boot in us */
extern int32_t (*scara_time_get)(void);

/* react to error, panic ! */
extern void (*scara_panic)(void);

#ifdef __cplusplus
}
#endif

#endif /* SCARA_PORT_H */
