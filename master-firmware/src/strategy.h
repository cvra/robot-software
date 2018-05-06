#ifndef STRATEGY_H
#define STRATEGY_H

#ifdef __cplusplus
extern "C" {
#endif

#include "robot_helpers/strategy_helpers.h"

void strategy_start(void);

bool strategy_goto_avoid(int x_mm, int y_mm, int a_deg, int traj_end_flags);

bool strat_check_distance_to_hand_lower_than(float expected_value);
bool strat_pick_cube(float x, float y);
bool strat_deposit_cube(float x, float y, int num_cubes_in_tower);

void strat_push_switch_on(float x, float y, float z, float y_push);
void strat_push_the_bee(point_t start, point_t end, float bee_height);
void strat_push_the_bee_v2(point_t start, float bee_height, float forward_motion);

void strat_collect_wastewater(enum strat_color_t color, float heading);
/** Returns true if succesful */
bool strat_fill_watertower(void);
void strat_fill_wastewater_treatment_plant(void);

bool strat_lever_is_full(enum lever_side_t lever_side);

#ifdef __cplusplus
}
#endif

#endif /* STRATEGY_H */
