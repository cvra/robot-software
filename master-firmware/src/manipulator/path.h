#ifndef MANIPULATOR_PATH_H
#define MANIPULATOR_PATH_H

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    MANIPULATOR_INIT = 0,
    MANIPULATOR_RETRACT,
    MANIPULATOR_DEPLOY,
    MANIPULATOR_LIFT_HORZ,
    MANIPULATOR_PICK_HORZ,
    MANIPULATOR_COUNT // Dummy, used for last element
} manipulator_state_t;

#ifdef __cplusplus
}
#endif

#endif /* MANIPULATOR_PATH_H */
