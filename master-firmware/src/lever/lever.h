#ifndef LEVER_H
#define LEVER_H

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    LEVER_DISABLED,
    LEVER_RETRACTED,
    LEVER_DEPLOYED,
} lever_state_t;

typedef struct {
    void (*set_lever)(void*, float);
    void* lever_args;

    lever_state_t state;
} lever_t;

/* Initialize lever */
void lever_init(lever_t* lever);

/* Actuator callbacks */
void lever_set_callbacks(lever_t* lever, void (*set_lever)(void*, float), void* lever_args);

/* Deploy lever */
void lever_deploy(lever_t* lever);

/* Retract lever */
void lever_retract(lever_t* lever);

#ifdef __cplusplus
}
#endif

#endif /* LEVER_H */
