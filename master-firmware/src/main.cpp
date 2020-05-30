#include <stdio.h>
#include <string.h>
#include <math.h>

#include <thread>
#include <absl/flags/flag.h>
#include <absl/flags/parse.h>
#include <absl/flags/usage.h>
#include <absl/synchronization/mutex.h>

#include "main.h"
#include "control_panel.h"
//#include <shell.h>
//#include "commands.h"
#include "debug/log.h"
#include "can/bus_enumerator.h"
#include <msgbus/posix/port.h>
#include "can/uavcan_node.h"
#include "config.h"
#include <parameter/parameter_msgpack.h>
#include <cmp_mem_access/cmp_mem_access.h>
#include "can/motor_manager.h"
//#include "lwipthread.h"
#include <error/error.h>
//#include "base/encoder.h"
#include "base/base_controller.h"
#include "robot_helpers/trajectory_helpers.h"
#include "strategy.h"
#include "gui.h"
//#include "udp_topic_broadcaster.h"
//#include "ally_position_service.h"
//
#include <aversive/trajectory_manager/trajectory_manager.h>

using namespace std::chrono_literals;

void init_base_motors();
void init_sensor_nodes();
void init_actuators();

motor_manager_t motor_manager;

/* Bus related declarations */
messagebus_t bus;
static MESSAGEBUS_POSIX_SYNC_DECL(bus_sync);

ABSL_FLAG(std::string, can_iface, "vcan0", "SocketCAN interface to use. If empty, disable UAVCAN.");
ABSL_FLAG(bool, verbose, false, "Enable verbose output");
ABSL_FLAG(bool, enable_gui, true, "Enable on-robot GUI");
ABSL_FLAG(std::string, robot_config, "simulation", "Which config to load, can be order, chaos or simulation.");

void config_load_err_cb(void* arg, const char* id, const char* err)
{
    (void)arg;
    ERROR("parameter %s: %s", id == NULL ? "(...)" : id, err);
}

void config_load_from_flash()
{
    cmp_ctx_t cmp;
    cmp_mem_access_t mem;

    extern unsigned char msgpack_config_chaos[];
    extern const size_t msgpack_config_chaos_size;
    extern unsigned char msgpack_config_order[];
    extern const size_t msgpack_config_order_size;
    extern unsigned char msgpack_config_simulation[];
    extern const size_t msgpack_config_simulation_size;

    if (absl::GetFlag(FLAGS_robot_config) == "chaos") {
        NOTICE("Loading msgpack config for Chaos");
        cmp_mem_access_ro_init(&cmp, &mem, msgpack_config_chaos, msgpack_config_chaos_size);
    } else if (absl::GetFlag(FLAGS_robot_config) == "order") {
        NOTICE("Loading msgpack config for order");
        cmp_mem_access_ro_init(&cmp, &mem, msgpack_config_order, msgpack_config_order_size);
    } else if (absl::GetFlag(FLAGS_robot_config) == "simulation") {
        NOTICE("Loading msgpack config for the simulator");
        cmp_mem_access_ro_init(&cmp, &mem, msgpack_config_simulation, msgpack_config_simulation_size);
    } else {
        ERROR("Unknown robot_config value %s", absl::GetFlag(FLAGS_robot_config).c_str());
    }

    int ret = parameter_msgpack_read_cmp(&global_config, &cmp, config_load_err_cb, nullptr);
    if (ret != 0) {
        ERROR("parameter_msgpack_read_cmp failed");
    }
}

static void blink_start()
{
    std::thread blink([]() {
        while (true) {
            control_panel_toggle(LED_PC);
            std::this_thread::sleep_for(500ms);
        }
    });
    blink.detach();
}

static void enable_deadlock_detection()
{
    absl::SetMutexDeadlockDetectionMode(absl::OnDeadlockCycle::kReport);
}

/** Application entry point.  */
int main(int argc, char** argv)
{
    enable_deadlock_detection();

    absl::SetProgramUsageMessage("Program responsible for running main"
                                 "functions of the robot");
    absl::ParseCommandLine(argc, argv);

    /* Initialize global objects. */
    config_init();

    blink_start();

    log_init(absl::GetFlag(FLAGS_verbose));

    NOTICE("boot");

    /* Initialize the interthread communication bus. */
    messagebus_init(&bus, &bus_sync, &bus_sync);

    // udp_topic_register_callbacks();

    /* bus enumerator init */
    struct bus_enumerator_entry_allocator bus_enum_entries_alloc[MAX_NB_BUS_ENUMERATOR_ENTRIES];

    bus_enumerator_init(&bus_enumerator,
                        bus_enum_entries_alloc,
                        MAX_NB_BUS_ENUMERATOR_ENTRIES);

    motor_driver_t motor_driver_buffer[MAX_NB_MOTOR_DRIVERS];

    motor_manager_init(&motor_manager,
                       motor_driver_buffer,
                       MAX_NB_MOTOR_DRIVERS,
                       &bus_enumerator);

    /* Initialize motors */
    init_base_motors();
    init_actuators();

    /* Initiaze UAVCAN communication */

    if (!absl::GetFlag(FLAGS_can_iface).empty()) {
        NOTICE("starting UAVCAN on %s", absl::GetFlag(FLAGS_can_iface).c_str());
        uavcan_node_start(absl::GetFlag(FLAGS_can_iface), 10);
    }

    /* Those service communicate over IP so must be started afterward */
    // udp_topic_broadcast_start();
    //ally_position_start();

    /* Load stored robot config */
    config_load_from_flash();

    control_panel_init();
    if (absl::GetFlag(FLAGS_enable_gui)) {
        gui_start();
    }

    /* Base init */
    robot_init();
    base_controller_start();
    position_manager_start();
    trajectory_manager_start();

    // Shell manager initialization.
    //shellInit();
    //   shell_spawn((BaseSequentialStream*)&SDU1);

    strategy_play_game();

    //trajectory_d_rel(&robot.traj, 300);
    /* just a simple test case */
    while (true) {
        std::this_thread::sleep_for(1s);
    }
}

void init_base_motors()
{
    motor_manager_create_driver(&motor_manager, "left-wheel");
    motor_manager_create_driver(&motor_manager, "right-wheel");
}

parameter_namespace_t actuators_ns;

actuator_driver_t actuator_front_left, actuator_front_center, actuator_front_right;
actuator_driver_t actuator_back_left, actuator_back_center, actuator_back_right;

void init_actuators()
{
    parameter_namespace_declare(&actuators_ns, &global_config, "actuators");

    actuator_driver_init(&actuator_front_left,
                         &bus_enumerator, &actuators_ns, "actuator-front-left");
    actuator_driver_init(&actuator_front_center,
                         &bus_enumerator, &actuators_ns, "actuator-front-center");
    actuator_driver_init(&actuator_front_right,
                         &bus_enumerator, &actuators_ns, "actuator-front-right");

    actuator_driver_init(&actuator_back_left,
                         &bus_enumerator, &actuators_ns, "actuator-back-left");
    actuator_driver_init(&actuator_back_center,
                         &bus_enumerator, &actuators_ns, "actuator-back-center");
    actuator_driver_init(&actuator_back_right,
                         &bus_enumerator, &actuators_ns, "actuator-back-right");
}
