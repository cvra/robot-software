
#ifndef RPM_H
#define RPM_H

#include <timestamp/timestamp.h>

#ifdef __cplusplus
extern "C" {
#endif

void rpm_barrier_crossing(timestamp_t time); // called by interrupt
float rpm_get_position(void);
float rpm_get_velocity(void);
void rpm_get_velocity_and_position(float* velocity, float* position);

#ifdef __cplusplus
}
#endif

#endif /* RPM_H */
