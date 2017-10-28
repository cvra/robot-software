#ifndef JOINT_H
#define JOINT_H

/** Joint data struct */
typedef struct {
    void (*set_position)(void*, float);
    void (*set_velocity)(void*, float);
    float (*get_position)(void*);
    void* args; /// Some internal state
} joint_t;

void joint_set_callbacks(joint_t *joint, void (*set_position)(void *, float),
                         void (*set_velocity)(void *, float),
                         float (*get_position)(void *), void *args);

#endif
