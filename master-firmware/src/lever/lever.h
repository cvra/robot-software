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
    lever_state_t state;
} lever_t;

/* Initialize lever */
void lever_init(lever_t* lever);

#ifdef __cplusplus
}
#endif

#endif /* LEVER_H */
