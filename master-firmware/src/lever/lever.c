#include <string.h>

#include "lever.h"

void lever_init(lever_t* lever)
{
    memset(lever, 0, sizeof(lever_t));

    lever->state = LEVER_DISABLED;
}
