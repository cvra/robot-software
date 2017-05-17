#ifndef PARAMETER_LISTENER_H
#define PARAMETER_LISTENER_H

#ifdef __cplusplus
extern "C" {
#endif

#include <ch.h>

void parameter_listener_start(BaseSequentialStream *dev);

#ifdef __cplusplus
}
#endif

#endif /* PARAMETER_LISTENER_H */
