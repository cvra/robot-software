#include <cvra/motor/EmergencyStop.hpp>
#include <uavcan_linux/uavcan_linux.hpp>

constexpr unsigned NodeMemoryPoolSize = 16384;

typedef uavcan::Node<NodeMemoryPoolSize> Node;

uavcan::ISystemClock& getSystemClock();
uavcan::ICanDriver& getCanDriver();

int main(int argc, char** argv)
{
    Node node(getCanDriver(), getSystemClock());
    node.setNodeID(42);
    node.setName("ch.cvra.motor-board-emulator");

    node.start();

    while (true) {
        /*
         * If there's nothing to do, the thread blocks inside the driver's
         * method select() until the timeout expires or an error occurs (e.g. driver failure).
         * All error codes are listed in the header uavcan/error.hpp.
         */
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
