#include <cvra/motor/control/Voltage.hpp>
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

class UavcanMotorEmulator {
    uavcan_linux::SystemClock clock;
    uavcan_linux::SocketCanDriver driver;
    std::unique_ptr<Node> node;
    std::thread thread_;

    using VoltageSub = uavcan::Subscriber<cvra::motor::control::Voltage>;
    std::unique_ptr<VoltageSub> voltage_sub;

public:
    UavcanMotorEmulator(std::string can_iface, std::string board_name, int node_number)
        : driver(clock)
    {
        NOTICE("Motor board emulator on %s", can_iface.c_str());
        if (driver.addIface(can_iface) < 0) {
            ERROR("Failed to add iface %s", can_iface.c_str());
        }
        node = std::make_unique<Node>(driver, clock);
        node->setHealthOk();
        node->setModeOperational();
        if (!node->setNodeID(node_number)) {
            ERROR("Invalid node number %d", node_number);
        }
        node->setName(board_name.c_str());

        voltage_sub = std::make_unique<VoltageSub>(*node);
        voltage_sub->start([&](const uavcan::ReceivedDataStructure<cvra::motor::control::Voltage>& msg) {
            DEBUG("Received voltage setpoint %.2f for board %d", msg.voltage, msg.node_id);
        });
    }

    void start()
    {
        std::thread new_thread(&UavcanMotorEmulator::spin, this);
        std::swap(new_thread, thread_);
    }

private:
    void spin()
    {
        node->start();
        while (true) {
            const int res = node->spin(uavcan::MonotonicDuration::fromMSec(1000));
            if (res < 0) {
                WARNING("UAVCAN failure: %d", res);
            }
        }
    }
};

int main(int argc, char** argv)
{
    absl::SetProgramUsageMessage("Emulates one of CVRA "
                                 "motor control board over UAVCAN");
    absl::ParseCommandLine(argc, argv);
    logging_init();

    int n = absl::GetFlag(FLAGS_first_uavcan_id);

    std::vector<std::unique_ptr<UavcanMotorEmulator>> emulators;
    for (auto name : absl::GetFlag(FLAGS_uavcan_names)) {
        auto emu = std::make_unique<UavcanMotorEmulator>(absl::GetFlag(FLAGS_can_iface), name, n);
        emulators.push_back(std::move(emu));
        n++;
    }

    for (auto& emu : emulators) {
        emu->start();
    }

    while (true) {
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }

    return 0;
}
