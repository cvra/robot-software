#ifndef BASE_H
#define BASE_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "motor_driver.h"

#define MOTOR_NAME_LEN 20
#define WHEEL_MAX_SPEED 6.0f // Maximum wheel speed achievable

typedef struct {
    motor_driver_t* motor;
    float speed_factor;
} wheel_t;

typedef struct {
    struct {
        wheel_t back_wheel;
        wheel_t center_wheel;
        wheel_t front_wheel;
    } left, right;
} base_t;

void base_set_speed(base_t* base, float linear_x, float angular_z);

#ifdef __cplusplus
}
#endif

#endif /* BASE_H */
