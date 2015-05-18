#ifndef STREAM_H
#define STREAM_H

#include <ch.h>

#define STREAM_PORT         20042
#define STREAM_TIMESTEP_MS  10

#ifdef __cplusplus
extern "C" {
#endif

void stream_init(void);

#ifdef __cplusplus
}
#endif

#endif /* STREAM_H */
