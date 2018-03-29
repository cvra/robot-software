#include <string.h>

#include "state.h"

bool operator==(const RobotState& lhs, const RobotState& rhs)
{
    return !memcmp(&lhs, &rhs, sizeof(RobotState));
}
