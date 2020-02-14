#include <error/error.h>
#include "actions.h"


bool actions::EnableLighthouse::execute(StrategyState &state)
{
    (void) state;
    NOTICE("Enabling the lighthouse");
    WARNING("not implemented yet");
    return false;
}

bool actions::RaiseWindsock::execute(StrategyState& state)
{
    (void) state;
    NOTICE("Raising windsock #%d", windsock_index);
    WARNING("not implemented yet");
    return false;
}


