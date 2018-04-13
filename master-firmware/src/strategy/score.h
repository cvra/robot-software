#ifndef SCORE_H
#define SCORE_H

#ifdef __cplusplus
extern "C" {
#endif

#include "strategy/state.h"

int score_count_bee_on_map(const RobotState& state);
int score_count_panel_on_map(const RobotState& state);

int score_count_bee(const RobotState& state);
int score_count_switch(const RobotState& state);

int score_count_tower(const RobotState& state);
int score_count_tower_bonus(const RobotState& state);

int score_count_balls(const RobotState& state);
int score_count_wastewater_bonus(const RobotState& state);

#ifdef __cplusplus
}
#endif

#endif /* SCORE_H */
