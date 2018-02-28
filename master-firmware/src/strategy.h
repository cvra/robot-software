#ifndef STRATEGY_H
#define STRATEGY_H

#ifdef __cplusplus
extern "C" {
#endif

void strategy_start(void);

bool strategy_goto_avoid(int x_mm, int y_mm, int a_deg, int traj_end_flags);

void strat_pick_cube(float x, float y, float z_start);
void strat_deposit_cube(float x, float y, int num_cubes_in_tower);

#ifdef __cplusplus
}
#endif

#endif /* STRATEGY_H */
