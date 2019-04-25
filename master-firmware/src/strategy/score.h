#ifndef SCORE_H
#define SCORE_H

#include "strategy/state.h"

#ifdef __cplusplus
extern "C" {
#endif

int score_count_red_atom_zone(const RobotState& state);

#ifdef __cplusplus
}
#endif

#endif /* SCORE_H */
