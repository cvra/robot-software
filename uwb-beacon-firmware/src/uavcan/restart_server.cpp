#include <uavcan/uavcan.hpp>
#include <uavcan/protocol/restart_request_server.hpp>
#include <ch.h>
#include "restart_server.hpp"

void restart_node(void *p)
{
    (void) p;
    NVIC_SystemReset();
}

class ChibiOSRestartHandler : public uavcan::IRestartRequestHandler {
    virtual_timer_t restart_timer;


    virtual bool handleRestartRequest(uavcan::NodeID request_source)
    {
        // Restarts the node after a delay to give enough time to send the
        // answer
        (void) request_source;
        chVTObjectInit(&restart_timer);
        chVTSet(&restart_timer, MS2ST(200), restart_node, NULL);
        return true;
    }
};

int restart_server_start(Node &node)
{
    static ChibiOSRestartHandler handler;
    node.setRestartRequestHandler(&handler);
    return 0;
}
