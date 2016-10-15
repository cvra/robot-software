#ifndef ROBOT_PARAMETERS_H
#define ROBOT_PARAMETERS_H

#ifdef __cplusplus
extern "C" {
#endif

#include <math.h>

/*
 * Robot base parameters
 */
#define ROBOT_EXTERNAL_WHEEL_RADIUS         0.016f
#define ROBOT_EXTERNAL_TRACK_LENGTH         0.194f

#define ROBOT_MOTOR_WHEEL_RADIUS            0.030f
#define ROBOT_MOTOR_TRACK_LENGTH            0.156f

#define EXTERNAL_ENCODER_TICKS_PER_TURN     16384

#define ROBOT_EXTERNAL_TRACK_LENGTH_MM      (1000. * ROBOT_EXTERNAL_TRACK_LENGTH)
#define EXTERNAL_ENCODER_TICKS_PER_MM       \
            (EXTERNAL_ENCODER_TICKS_PER_TURN \
                / (2. * 1000. * ROBOT_EXTERNAL_WHEEL_RADIUS * M_PI))

#define LEFT_WHEEL_CORRECTION_FACTOR        -0.99952566
#define RIGHT_WHEEL_CORRECTION_FACTOR       1.00047433

#ifdef __cplusplus
}
#endif

#endif /* ROBOT_PARAMETERS_H */
