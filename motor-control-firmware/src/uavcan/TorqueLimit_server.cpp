#include <cvra/motor/config/TorqueLimit.hpp>
#include "TorqueLimit_server.hpp"
#include "main.h"

int  TorqueLimit_server_start(Node &node)
{
    int ret;

    static uavcan::ServiceServer<cvra::motor::config::TorqueLimit> load_config_srv(node);

    ret = load_config_srv.start(
        [&](const uavcan::ReceivedDataStructure<cvra::motor::config::TorqueLimit::Request>&
            req,
            cvra::motor::config::TorqueLimit::Response& rsp)
    {

        (void) rsp;
        parameter_scalar_set(parameter_find(&parameter_root_ns, "/control/torque_limit"),
                             req.torque_limit);

    });

    return ret;
}
