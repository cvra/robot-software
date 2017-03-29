#include <cvra/motor/config/CurrentPID.hpp>
#include <ch.h>
#include <hal.h>
#include <chprintf.h>
#include <control.h>
#include "main.h"
#include "CurrentPID_server.hpp"

static void set_param(const char *name, const float value)
{
    parameter_t *p = parameter_find(&parameter_root_ns, name);
    parameter_scalar_set(p, value);
}

int CurrentPID_server_start(Node &node)
{
    int ret;

    static uavcan::ServiceServer<cvra::motor::config::CurrentPID> load_config_srv(node);

    ret = load_config_srv.start(
        [&](const uavcan::ReceivedDataStructure<cvra::motor::config::CurrentPID::Request>&
            req,
            cvra::motor::config::CurrentPID::Response& rsp)
    {

        (void) rsp;     /* empty response */
        set_param("/control/current/kp", req.pid.kp);
        set_param("/control/current/ki", req.pid.ki);
        set_param("/control/current/kd", req.pid.kd);
        set_param("/control/current/i_limit", req.pid.ilimit);
    });

    return ret;
}
