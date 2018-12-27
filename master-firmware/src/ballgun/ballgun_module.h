#ifndef BALLGUN_MODULE_H
#define BALLGUN_MODULE_H

#include "ballgun.h"

#ifdef __cplusplus
extern "C" {
#endif

#define BALLGUN_FREQUENCY 10

extern ballgun_t main_ballgun;

void ballgun_module_start(void);
void ball_accelerator_voltage(float voltage);
void ball_accelerator_velocity(float velocity);

#ifdef __cplusplus
}
#endif

#endif /* BALLGUN_MODULE_H */
