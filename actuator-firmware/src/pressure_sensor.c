#include "pressure_sensor.h"

void mpr_start_measurement(mpr_driver_t* drv)
{
    const uint8_t cmd[] = {0xaa, 0x00, 0x00};
    uint8_t output[sizeof(cmd)];

    drv->select(drv->arg);
    drv->transmit(drv->arg, cmd, output, sizeof(cmd));
    drv->unselect(drv->arg);
}

uint8_t mpr_read_status(mpr_driver_t* drv)
{
    uint8_t result;
    uint8_t cmd = 0xf0; // no-op

    drv->select(drv->arg);
    drv->transmit(drv->arg, &cmd, &result, 1);
    drv->unselect(drv->arg);

    return result;
}

uint32_t mpr_read_data(mpr_driver_t* drv)
{
    uint32_t result = 0;
    const uint8_t cmd[] = {0xf0, 0x00, 0x00, 0x00};
    uint8_t output[sizeof(cmd)];

    drv->select(drv->arg);
    drv->transmit(drv->arg, cmd, output, sizeof(cmd));
    drv->unselect(drv->arg);

    result |= output[3] << 0;
    result |= output[2] << 8;
    result |= output[1] << 16;

    return result;
}

int mpr_status_is_error(uint8_t status)
{
    /* Check if device is powered off */
    if ((status & (1 << 6)) == 0) {
        return 1;
    }

    /* Check if device has a memory error */
    if ((status & (1 << 2))) {
        return 1;
    }

    /* Check if device has a math error */
    if ((status & (1 << 0))) {
        return 1;
    }

    /* Check if unused bits are set to 0 */
    if ((status & 0b10011000)) {
        return 1;
    }

    return 0;
}

/** Returns true if the given device status represents a busy device. */
int mpr_status_is_busy(uint8_t status)
{
    return (status & (1 << 5));
}

float mpr_pressure_raw_to_pascal(uint32_t raw_pressure)
{
    /* See page 19 of the MPR datasheet */
    const float p_max = 103422; /* 15 psi, in pascal */
    const float out_max = 0xe66666, out_min = 0x19999A;

    return (raw_pressure - out_min) / (out_max - out_min) * p_max;
}
