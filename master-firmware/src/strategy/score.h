#ifndef SCORE_H
#define SCORE_H

#include "strategy/state.h"

#ifdef __cplusplus
extern "C" {
#endif

int score_count_classified_atoms(const RobotState& state);
int score_count_accelerator(const RobotState& state);
int score_count_goldenium(const RobotState& state);
int score_count_experiment(const RobotState& state);
int score_count_electron(const RobotState& state);

#ifdef __cplusplus
}
#endif

#endif /* SCORE_H */
