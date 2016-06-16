#ifndef POSITION_MANAGER_H
#define POSITION_MANAGER_H

#ifdef __cplusplus
extern "C" {
#endif

void position_manager_start(void);

void position_manager_reset(float x, float y, float heading);


#ifdef __cplusplus
}
#endif

#endif /* POSITION_MANAGER_H */
