#include <memory>
#include <uavcan/uavcan.hpp>
#include <uavcan/protocol/global_time_sync_master.hpp>
#include <error/error.h>
#include "time_sync_server.h"

static std::unique_ptr<uavcan::GlobalTimeSyncMaster> master;
static std::unique_ptr<uavcan::Timer> sync_timer;

int time_sync_server_start(uavcan::INode& node)
{
    master = std::make_unique<uavcan::GlobalTimeSyncMaster>(node);
    sync_timer = std::make_unique<uavcan::Timer>(node);
    sync_timer->setCallback([&](const uavcan::TimerEvent& /*unused*/) {
        DEBUG("publishing timesync");
        int res = master->publish();
        if (res < 0) {
            WARNING("Time sync master error: %d\n", res);
        }
    });

    sync_timer->startPeriodic(uavcan::MonotonicDuration::fromMSec(1000));
    return 0;
}
