#include <string.h>
#include "motor_driver.h"


void motor_driver_init(motor_driver_t *d,
                       const char *actuator_id,
                       parameter_namespace_t *ns)
{
    strncpy(d->id, actuator_id, MOTOR_ID_MAX_LEN);
    d->id[MOTOR_ID_MAX_LEN] = '\0';
    d->can_id = CAN_ID_NOT_SET;
    d->control_mode = MOTOR_CONTROL_MODE_DISABLED;
}

