#include <hal.h>
#include "index.h"
#include "control.h"

float position;

static void exti_callback(EXTDriver *extp, expchannel_t channel);

static const EXTConfig extcfg = {{
    {EXT_CH_MODE_DISABLED, NULL}, // 0
    {EXT_CH_MODE_DISABLED, NULL}, // 1
    {EXT_CH_MODE_DISABLED, NULL}, // 2
    {EXT_CH_MODE_DISABLED, NULL}, // 3
    {EXT_CH_MODE_DISABLED, NULL}, // 4
    {EXT_CH_MODE_DISABLED, NULL}, // 5
    {EXT_CH_MODE_DISABLED, NULL}, // 6
    {EXT_CH_MODE_DISABLED, NULL}, // 7
    {EXT_CH_MODE_DISABLED, NULL}, // 8
    {EXT_CH_MODE_DISABLED, NULL}, // 9
    {EXT_CH_MODE_DISABLED, NULL}, // 10
    {EXT_CH_MODE_DISABLED, NULL}, // 11
    // Index pin, active low, PA12
    {EXT_CH_MODE_FALLING_EDGE | EXT_CH_MODE_AUTOSTART | EXT_MODE_GPIOA, exti_callback},
    {EXT_CH_MODE_DISABLED, NULL}, // 13
    {EXT_CH_MODE_DISABLED, NULL}, // 14
    {EXT_CH_MODE_DISABLED, NULL}, // 15
    {EXT_CH_MODE_DISABLED, NULL}, // 16
    {EXT_CH_MODE_DISABLED, NULL}, // 17
    {EXT_CH_MODE_DISABLED, NULL}, // 18
    {EXT_CH_MODE_DISABLED, NULL}, // 19
    {EXT_CH_MODE_DISABLED, NULL}, // 20
    {EXT_CH_MODE_DISABLED, NULL}, // 21
    {EXT_CH_MODE_DISABLED, NULL}  // 22
}};

static void exti_callback(EXTDriver *extp, expchannel_t channel)
{
    (void)extp;
    (void)channel;

    position = control_get_position();
}

void index_init(void)
{
    position = 0;
    extStart(&EXTD1, &extcfg);
}

float index_get_position(void)
{
    return position;
}
