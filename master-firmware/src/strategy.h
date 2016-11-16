#ifndef STRATEGY_H
#define STRATEGY_H

#ifdef __cplusplus
extern "C" {
#endif

void strategy_start(void);

void strategy_goto_avoid(struct _robot* robot, int x_mm, int y_mm, int a_deg, int num_retries);

#ifdef __cplusplus
}
#endif

#endif /* STRATEGY_H */
