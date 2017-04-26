#ifndef STRATEGY_H
#define STRATEGY_H

#ifdef __cplusplus
extern "C" {
#endif

void strategy_start(void);

bool strategy_goto_avoid(int x_mm, int y_mm, int a_deg, int traj_end_flags);

#ifdef __cplusplus
}
#endif

#endif /* STRATEGY_H */
