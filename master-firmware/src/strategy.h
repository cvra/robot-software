#ifndef STRATEGY_H
#define STRATEGY_H

#include "robot_helpers/strategy_helpers.h"

#ifdef __cplusplus
extern "C" {
#endif

void strategy_start(void);

bool strategy_goto_avoid(int x_mm, int y_mm, int a_deg, int traj_end_flags);

#ifdef __cplusplus
}
#endif

#endif /* STRATEGY_H */
