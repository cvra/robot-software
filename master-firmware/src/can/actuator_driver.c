#include <string.h>
#include <can/actuator_driver.h>

void actuator_driver_init(actuator_driver_t* drv,
                          bus_enumerator_t* be,
                          parameter_namespace_t* parent,
                          const char* can_name)
{
    memset(drv, 0, sizeof(actuator_driver_t));

    strncpy(drv->can_name, can_name, ACTUATOR_NAME_MAX_LENGTH);

    parameter_namespace_declare(&drv->param_ns, parent, drv->can_name);

    parameter_scalar_declare_with_default(&drv->high_position, &drv->param_ns, "high_position", 0.001);
    parameter_scalar_declare_with_default(&drv->reef_pickup_position, &drv->param_ns, "reef_pickup_position", 0.002);
    parameter_scalar_declare_with_default(&drv->table_pickup_position, &drv->param_ns, "table_pickup_position", 0.003);
    parameter_scalar_declare_with_default(&drv->pump_duty_cycle, &drv->param_ns, "pump_pwm", 0.3);

    drv->actuator_position = POSITION_HIGH;

    bus_enumerator_add_node(be, can_name, (void*)drv);
}

void actuator_driver_set_position(actuator_driver_t* drv, enum actuator_position_enum pos)
{
    drv->actuator_position = pos;
}

void actuator_driver_pump_set(actuator_driver_t* drv, enum actuator_pump_enum pump, bool enabled)
{
    drv->pump_enabled[pump] = enabled;
}

bool actuator_driver_prepare_uavcan_msg(actuator_driver_t* drv,
                                        bus_enumerator_t* be,
                                        int* node_id,
                                        float* pump0,
                                        float* pump1,
                                        bool* solenoid0,
                                        bool* solenoid1,
                                        float* servo)
{
    *node_id = bus_enumerator_get_can_id(be, drv->can_name);

    /* If the node id is not valid, return immediately. */
    if (*node_id > 127) {
        return false;
    }

    *pump0 = parameter_scalar_get(&drv->pump_duty_cycle) * drv->pump_enabled[0];
    *pump1 = parameter_scalar_get(&drv->pump_duty_cycle) * drv->pump_enabled[1];

    /* Solenoids must be enabled at the same time as pumps */
    *solenoid0 = drv->pump_enabled[0];
    *solenoid1 = drv->pump_enabled[1];

    switch (drv->actuator_position) {
        case POSITION_HIGH:
            *servo = parameter_scalar_get(&drv->high_position);
            break;
        case POSITION_REEF_PICKUP:
            *servo = parameter_scalar_get(&drv->reef_pickup_position);
            break;
        case POSITION_TABLE_PICKUP:
            *servo = parameter_scalar_get(&drv->table_pickup_position);
            break;
    }

    return true;
}
