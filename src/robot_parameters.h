#ifndef ROBOT_PARAMETERS_H
#define ROBOT_PARAMETERS_H

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Robot base parameters
 */
#define ROBOT_EXTERNAL_WHEEL_RADIUS         0.016f
#define ROBOT_EXTERNAL_TRACK_LENGTH         0.194f

#define ROBOT_MOTOR_WHEEL_RADIUS            0.030f
#define ROBOT_MOTOR_TRACK_LENGTH            0.156f

#define EXTERNAL_ENCODER_TICKS_PER_TURN     16384

#ifdef __cplusplus
}
#endif

#endif /* ROBOT_PARAMETERS_H */
