#ifndef ACTUATOR_DRIVER_H
#define ACTUATOR_DRIVER_H

#include <parameter/parameter.h>
#include "can/bus_enumerator.h"

#define ACTUATOR_NAME_MAX_LENGTH 32

enum actuator_position_enum {
    /** Position used to pickup cups on the table. */
    POSITION_TABLE_PICKUP = 0,

    /** Position used to pickup cups on the reefs/dispensers. */
    POSITION_REEF_PICKUP,

    /** Highest position, to store the cup in the robot. */
    POSITION_HIGH,
};

/** Designates one of the pumps on the arms (one per suction cup). */
enum actuator_pump_enum {
    PUMP_TOP = 0,
    PUMP_FRONT = 1,
    PUMP_COUNT,
};

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    char can_name[ACTUATOR_NAME_MAX_LENGTH + 1];
    parameter_namespace_t param_ns;

    /* Various parameters related to how the hardware should be handled. */
    parameter_t table_pickup_position; ///< Duty cycle for POSITION_TABLE_PICKUP
    parameter_t reef_pickup_position; ///< Duty cycle for POSITION_REEF_PICKUP
    parameter_t high_position; ///< Duty cycle for POSITION_HIGH
    parameter_t pump_duty_cycle; ///< Duty cycle of the pumps in the "on" position.

    /* State of the actuator module (outgoing) */
    enum actuator_position_enum actuator_position;
    bool pump_enabled[PUMP_COUNT];
} actuator_driver_t;

/** Initializes an actuator driver.
 *
 * can_name is the node name, must match what is stored in the bus_enumerator.
 * parent is where the parameters will be stored. For example, If you provide
 * "/actuators", and can_name is "front-left", the parameters will be under
 * "/actuators/front-left/".
 *
 * Also registers the expected node name in the bus enumerator.
 */
void actuator_driver_init(actuator_driver_t* drv,
                          bus_enumerator_t* be,
                          parameter_namespace_t* parent,
                          const char* can_name);

/** Sets position of the pickup arm. */
void actuator_driver_set_position(actuator_driver_t* drv, enum actuator_position_enum pos);

/** Turns the given pump on or off, along with the associated solenoid. */
void actuator_driver_pump_set(actuator_driver_t* drv, enum actuator_pump_enum pump, bool enabled);

/** Reads the data required for a single cvra.actuator.Command UAVCAN message.
 *
 * Returns true if the message can be sent.
 */
bool actuator_driver_prepare_uavcan_msg(actuator_driver_t* drv,
                                        bus_enumerator_t* be,
                                        int* node_id,
                                        float* pump0,
                                        float* pump1,
                                        bool* solenoid0,
                                        bool* solenoid1,
                                        float* servo);

#ifdef __cplusplus
}
#endif
#endif
