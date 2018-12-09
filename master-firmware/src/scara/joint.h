#ifndef JOINT_H
#define JOINT_H

/** Joint data struct */
typedef struct {
    void (*set_position)(void*, float);
    void (*set_velocity)(void*, float);
    float (*get_position)(void*);
    void* args; /// Some internal state
} joint_t;

void joint_set_callbacks(joint_t* joint, void (*set_position)(void*, float), void (*set_velocity)(void*, float), float (*get_position)(void*), void* args);

/** Joint setpoint mode */
typedef enum {
    POSITION,
    VELOCITY
} joint_mode_t;

/** Joint setpoint */
typedef struct {
    joint_mode_t mode;
    float value;
} joint_setpoint_t;

void joint_set(joint_t* joint, joint_setpoint_t setpoint);

#endif
