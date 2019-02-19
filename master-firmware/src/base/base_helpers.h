#ifndef BASE_HELPERS_H
#define BASE_HELPERS_H

#include <aversive/position_manager/position_manager.h>
#include "math/lie_groups.h"

#ifdef __cplusplus
extern "C" {
#endif

se2_t base_get_robot_pose(struct robot_position* pos);

#ifdef __cplusplus
}
#endif

#endif /* BASE_HELPERS_H */
