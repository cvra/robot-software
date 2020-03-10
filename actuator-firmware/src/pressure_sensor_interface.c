#include "board.h"
#include "pressure_sensor_interface.h"
#include <ch.h>
#include <hal.h>

static void mpr_select(void* arg)
{
    spiAcquireBus(&SPID3);
    if ((int)arg == 0) {
        palClearPad(GPIOA, GPIOA_CS1);
    } else {
        palClearPad(GPIOA, GPIOA_CS2);
    }
}

static void mpr_unselect(void* arg)
{
    if ((int)arg == 0) {
        palSetPad(GPIOA, GPIOA_CS1);
    } else {
        palSetPad(GPIOA, GPIOA_CS2);
    }
    spiAcquireBus(&SPID3);
}

static void mpr_transmit(void* arg, const uint8_t* tx, uint8_t* rx, size_t n)
{
    (void)arg;
    spiExchange(&SPID3, n, tx, rx);
}

mpr_driver_t pressure_sensors[2] = {{.arg = (void*)0,
                                     .transmit = mpr_transmit,
                                     .select = mpr_select,
                                     .unselect = mpr_unselect},
                                    {.arg = (void*)1,
                                     .transmit = mpr_transmit,
                                     .select = mpr_select,
                                     .unselect = mpr_unselect}};
