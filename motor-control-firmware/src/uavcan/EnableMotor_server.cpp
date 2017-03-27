#include <cvra/motor/config/EnableMotor.hpp>
#include <ch.h>
#include <hal.h>
#include <chprintf.h>
#include <control.h>
#include "main.h"
#include "EnableMotor_server.hpp"

int EnableMotor_server_start(Node &node)
{
    int ret;

    static uavcan::ServiceServer<cvra::motor::config::EnableMotor> load_config_srv(node);

    ret = load_config_srv.start(
        [&](const uavcan::ReceivedDataStructure<cvra::motor::config::EnableMotor::Request>&
            req,
            cvra::motor::config::EnableMotor::Response& rsp)
    {
        (void) rsp;         /* empty response */
        control_enable(req.enable);
    });

    return ret;
}
