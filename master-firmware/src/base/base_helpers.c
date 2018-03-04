#include "base_helpers.h"

se2_t base_get_robot_pose(struct robot_position* pos)
{
    const float x = position_get_x_float(pos);
    const float y = position_get_y_float(pos);
    const float a = position_get_a_rad_float(pos);
    return se2_create_xya(x, y, a);
}
