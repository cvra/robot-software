#include <cvra/motor/EmergencyStop.hpp>
#include <uavcan_linux/uavcan_linux.hpp>
#include "absl/flags/flag.h"
#include "absl/flags/parse.h"
#include "absl/flags/usage.h"

constexpr unsigned NodeMemoryPoolSize = 16384;

typedef uavcan::Node<NodeMemoryPoolSize> Node;

ABSL_FLAG(int, uavcan_id, 42, "UAVCAN ID of this instance");
ABSL_FLAG(std::string, uavcan_name, "example-board", "UAVCAN board name of this instance");

uavcan::ISystemClock& getSystemClock();
uavcan::ICanDriver& getCanDriver();

int main(int argc, char** argv)
{
    absl::SetProgramUsageMessage("Emulates one of CVRA "
                                 "motor control board over UAVCAN");
    absl::ParseCommandLine(argc, argv);

    Node node(getCanDriver(), getSystemClock());

    node.setNodeID(absl::GetFlag(FLAGS_uavcan_id));
    node.setName(absl::GetFlag(FLAGS_uavcan_name).c_str());

    node.start();

    while (true) {
        const int res = node.spin(uavcan::MonotonicDuration::fromMSec(1000));
        if (res < 0) {
            std::cerr << "Transient failure: " << res << std::endl;
        }
    }

    return 0;
}

uavcan::ISystemClock& getSystemClock()
{
    static uavcan_linux::SystemClock clock;
    return clock;
}

uavcan::ICanDriver& getCanDriver()
{
    static uavcan_linux::SocketCanDriver driver(dynamic_cast<const uavcan_linux::SystemClock&>(getSystemClock()));
    if (driver.getNumIfaces() == 0) {
        if (driver.addIface("vcan0") < 0) {
            throw std::runtime_error("Failed to add iface");
        }
    }
    return driver;
}
