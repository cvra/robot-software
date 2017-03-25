#ifndef SCARA_PORT_H
#define SCARA_PORT_H

#include <stdint.h>

/* returns time since boot in us */
extern int32_t (*scara_time_get)(void);

/* react to error, panic ! */
extern void (*scara_panic)(void);

#endif /* SCARA_PORT_H */
