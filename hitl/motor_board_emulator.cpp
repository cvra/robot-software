#include <cvra/motor/EmergencyStop.hpp>
#include <uavcan_linux/uavcan_linux.hpp>
#include <thread>
#include <vector>
#include "absl/flags/flag.h"
#include "absl/flags/parse.h"
#include "absl/flags/usage.h"
#include "error/error.h"
#include "logging.h"

constexpr unsigned NodeMemoryPoolSize = 16384;

typedef uavcan::Node<NodeMemoryPoolSize> Node;

ABSL_FLAG(int, first_uavcan_id, 42, "UAVCAN ID of the first board."
                                    " Subsequent ones will be incremented by 1 each.");
ABSL_FLAG(std::vector<std::string>, uavcan_names, {"example-board"}, "Comma separated list of motor boards to emulate.");
ABSL_FLAG(std::string, can_iface, "vcan0", "SocketCAN interface to connect the emulation to");

uavcan::ISystemClock& getSystemClock();
uavcan::ICanDriver& getCanDriver();

void node_thread(int id, std::string board_name)
{
    NOTICE("Starting CAN thread for '%s'", board_name.c_str());
    uavcan_linux::SystemClock clock;
    uavcan_linux::SocketCanDriver driver(clock);

    auto iface = absl::GetFlag(FLAGS_can_iface).c_str();
    if (driver.addIface(iface) < 0) {
        ERROR("Failed to add iface %s", iface);
    }

    Node node(driver, clock);
    node.setNodeID(id);
    node.setName(board_name.c_str());

    node.start();

    while (true) {
        const int res = node.spin(uavcan::MonotonicDuration::fromMSec(1000));
        if (res < 0) {
            WARNING("UAVCAN failure (%s): %d", board_name.c_str(), res);
        }
    }
}

int main(int argc, char** argv)
{
    absl::SetProgramUsageMessage("Emulates one of CVRA "
                                 "motor control board over UAVCAN");
    absl::ParseCommandLine(argc, argv);
    logging_init();

    int n = absl::GetFlag(FLAGS_first_uavcan_id);

    std::vector<std::thread> threads;

    for (auto& name : absl::GetFlag(FLAGS_uavcan_names)) {
        std::thread t(node_thread, n, name);
        threads.push_back(std::move(t));
        n++;
    }

    for (auto& thread : threads) {
        thread.join();
    }

    return 0;
}
