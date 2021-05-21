#pragma once

#include <cstdint>
#include "strategy/color.h"

/** Auto position robot at requested location, and ensure the correct
 *  position is reached by aligning against walls.
 */
void strategy_auto_position(int32_t x, int32_t y, int32_t heading, enum strat_color_t robot_color);

/** Align on y axis */
void strategy_align_y(int32_t y);
