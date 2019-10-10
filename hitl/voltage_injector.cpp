#include <cvra/motor/control/Voltage.hpp>
#include <uavcan_linux/uavcan_linux.hpp>
#include "absl/flags/flag.h"
#include "absl/flags/parse.h"
#include "absl/flags/usage.h"
#include "error/error.h"
#include "logging.h"

typedef uavcan::Node<16384> Node;

ABSL_FLAG(int, uavcan_id, 10, "UAVCAN ID to use");
ABSL_FLAG(int, dst_id, 42, "ID of the board to which send messages");
ABSL_FLAG(std::string, can_iface, "vcan0", "SocketCAN interface to connect the emulation to");

int main(int argc, char** argv)
{
    absl::SetProgramUsageMessage("Sends a voltage setpoint");
    absl::ParseCommandLine(argc, argv);
    logging_init();

    uavcan_linux::SystemClock clock;
    uavcan_linux::SocketCanDriver driver(clock);

    driver.addIface(absl::GetFlag(FLAGS_can_iface).c_str());

    Node node(driver, clock);

    node.setNodeID(absl::GetFlag(FLAGS_uavcan_id));
    node.setName("ch.cvra.hitl.voltage_injector");
    node.setHealthOk();
    node.setModeOperational();

    node.start();

    uavcan::Publisher<cvra::motor::control::Voltage> pub(node);
    auto res = pub.init();
    if (res < 0) {
        ERROR("Could not init publisher (error %d)", res);
    }

    while (true) {
        cvra::motor::control::Voltage voltage;
        voltage.voltage = 12.;
        voltage.node_id = absl::GetFlag(FLAGS_dst_id);
        pub.broadcast(voltage);
        node.spin(uavcan::MonotonicDuration::fromMSec(1000));
    }
}
