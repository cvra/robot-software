#include <absl/synchronization/mutex.h>
#include <cvra/motor/control/Voltage.hpp>
#include <cvra/odometry/WheelEncoder.hpp>
#include <uavcan_linux/uavcan_linux.hpp>
#include <thread>
#include "absl/flags/flag.h"
#include "absl/flags/parse.h"
#include "absl/flags/usage.h"
#include "motor_board_emulator.h"
#include "wheel_encoders_emulator.h"
#include <error/error.h>
#include "logging.h"

ABSL_FLAG(int, first_uavcan_id, 42, "UAVCAN ID of the first board."
                                    " Subsequent ones will be incremented by 1 each.");
ABSL_FLAG(std::string, can_iface, "vcan0", "SocketCAN interface to connect the emulation to");

int main(int argc, char** argv)
{
    absl::SetProgramUsageMessage("Emulates one of CVRA "
                                 "motor control board over UAVCAN");
    absl::ParseCommandLine(argc, argv);
    logging_init();

    int board_id = absl::GetFlag(FLAGS_first_uavcan_id);
    std::string iface = absl::GetFlag(FLAGS_can_iface);

    UavcanMotorEmulator left_motor(iface, "left-wheel", board_id++);
    UavcanMotorEmulator right_motor(iface, "right-wheel", board_id++);
    WheelEncoderEmulator wheels(iface, "encoders", board_id++);

    right_motor.start();
    left_motor.start();
    wheels.start();

    float left = 0.f, right = 0.f;
    while (true) {
        const float dt = 0.01;
        std::this_thread::sleep_for(dt * std::chrono::seconds(1));

        // fake value, for testing
        const float speed_const = 400;
        left += left_motor.get_voltage() * dt * speed_const;
        right += right_motor.get_voltage() * dt * speed_const;

        wheels.set_encoders(left, right);
    }

    return 0;
}
