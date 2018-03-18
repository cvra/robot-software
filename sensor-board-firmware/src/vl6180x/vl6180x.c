#include <stdbool.h>
#include <stddef.h>
#include "vl6180x.h"
#include "vl6180x_registers.h"

bool vl6180x_ping(vl6180x_t *dev)
{
    uint8_t id = vl6180x_read_register(dev, VL6180X_IDENTIFICATION_MODEL_ID);
    if (0xB4 == id) {
        return true;
    } else {
        return false;
    }
}

void vl6180x_init(vl6180x_t *dev, void *i2c_dev, uint8_t address)
{
    dev->i2c = i2c_dev;
    dev->address = address;
}

uint8_t vl6180x_measure_distance(vl6180x_t *dev, uint8_t *out_mm)
{
    uint8_t status, mm;

    /* Wait for device ready. */
    do {
        status = vl6180x_read_register(dev, VL6180X_RESULT_RANGE_STATUS);
    } while ((status & (1 << 0)) == 0);

    /* Start measurement. */
    vl6180x_write_register(dev, VL6180X_SYSRANGE_START, 0x01);

    /* Wait for measurement ready. */
    do {
        status = vl6180x_read_register(dev, VL6180X_RESULT_INTERRUPT_STATUS_GPIO);
    } while ((status & (1 << 2)) == 0);

    /* Read result. */
    mm = vl6180x_read_register(dev, VL6180X_RESULT_RANGE_VAL);
    *out_mm = mm;

    /* Clear interrupt flags. */
    vl6180x_write_register(dev, VL6180X_SYSTEM_INTERRUPT_CLEAR, 0x07);

    /* Wait for device ready. */
    do {
        status = vl6180x_read_register(dev, VL6180X_RESULT_RANGE_STATUS);
    } while ((status & (1 << 0)) == 0);

    /* Return error code. */
    return status >> 4;
}

void vl6180x_change_i2c_address(vl6180x_t *dev, uint8_t address)
{
    vl6180x_write_register(dev, VL6180X_I2C_SLAVE_DEVICE_ADDRESS, address);
    dev->address = address;
}

/* Configuration as in application note AN4545: VL6180X basic ranging */
void vl6180x_configure(vl6180x_t *dev)
{
    while (vl6180x_read_register(dev, 0x16) != 0x01) {
    }
    /* clear SYSTEM__FRESH_OUT_OF_RESET */
    vl6180x_write_register(dev, 0x016, 0x00);

    static const struct {
        uint16_t reg_addr;
        uint8_t value;
    } init_tab[] = {
    /* Mandatory: Private registers. */
    {0x0207, 0x01}, {0x0208, 0x01}, {0x0096, 0x00}, {0x0097, 0xfd},
    {0x00e3, 0x00}, {0x00e4, 0x04}, {0x00e5, 0x02}, {0x00e6, 0x01},
    {0x00e7, 0x03}, {0x00f5, 0x02}, {0x00d9, 0x05}, {0x00db, 0xce},
    {0x00dc, 0x03}, {0x00dd, 0xf8}, {0x009f, 0x00}, {0x00a3, 0x3c},
    {0x00b7, 0x00}, {0x00bb, 0x3c}, {0x00b2, 0x09}, {0x00ca, 0x09},
    {0x0198, 0x01}, {0x01b0, 0x17}, {0x01ad, 0x00}, {0x00ff, 0x05},
    {0x0100, 0x05}, {0x0199, 0x05}, {0x01a6, 0x1b}, {0x01ac, 0x3e},
    {0x01a7, 0x1f}, {0x0030, 0x00},
    /* Recommended : Public registers - See data sheet for more detail */
    /* Enables polling for New Sample ready when measurement completes */
    {0x0011, 0x10},
    /* Set the averaging sample period (compromise between lower noise and
     * increased execution time) */
    {0x010a, 0x30},
    /* Sets the light and dark gain (upper nibble). Dark gain should not be
     * changed.*/
    {0x003f, 0x46},
    /* sets the # of range measurements after which auto calibration of system
     * is performed */
    {0x0031, 0xFF},
    /* Set ALS integration time to 100ms */
    {0x0040, 0x63},
    /* perform a single temperature calibration of the ranging sensor */
    {0x002e, 0x01},
    /* Configure interrupt on new sample ready. Required for polling to work. */
    {0x0014, 0x24},
    };

    const size_t init_tab_len = sizeof(init_tab)/sizeof(init_tab[0]);
    size_t i;
    for (i = 0; i < init_tab_len; i++) {
        vl6180x_write_register(dev, init_tab[i].reg_addr, init_tab[i].value);
    }
}
