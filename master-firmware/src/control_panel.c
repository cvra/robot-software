#include <hal.h>
#include "control_panel.h"

struct control_panel_io {
    stm32_gpio_t* port;
    uint16_t pin;
};

static struct control_panel_io input_pins[] = {
    {GPIOE, GPIOE_EXT_IO_8}, // BUTTON_YELLOW
    {GPIOF, GPIOF_EXT_IO_9}, // BUTTON_GREEN
    {GPIOG, GPIOG_EXT_IO_10}, // STARTER
};

static struct control_panel_io output_pins[] = {
    {GPIOE, GPIOE_EXT_IO_0}, // LED_READY
    {GPIOG, GPIOG_EXT_IO_1}, // LED_DEBUG
    {GPIOD, GPIOD_EXT_IO_2}, // LED_ERROR
    {GPIOG, GPIOG_EXT_IO_3}, // LED_POWER
    {GPIOG, GPIOG_EXT_IO_4}, // LED_PC
    {GPIOF, GPIOF_EXT_IO_5}, // LED_BUS
    {GPIOF, GPIOF_EXT_IO_6}, // LED_YELLOW
    {GPIOF, GPIOF_EXT_IO_7}, // LED_GREEN
};

void control_panel_init(void)
{
    for (unsigned i = 0; i < sizeof(output_pins)/sizeof(output_pins[0]); i++) {
        palSetPadMode(output_pins[i].port, output_pins[i].pin, PAL_MODE_OUTPUT_PUSHPULL);
    }

    for (unsigned i = 0; i < sizeof(input_pins)/sizeof(input_pins[0]); i++) {
        palSetPadMode(input_pins[i].port, input_pins[i].pin, PAL_MODE_INPUT_PULLDOWN);
    }
}

bool control_panel_read(enum control_panel_input in)
{
    return palReadPad(input_pins[in].port, input_pins[in].pin);
}

void control_panel_set(enum control_panel_output out)
{
    palClearPad(output_pins[out].port, output_pins[out].pin);
}

void control_panel_clear(enum control_panel_output out)
{
    palSetPad(output_pins[out].port, output_pins[out].pin);
}

void control_panel_toggle(enum control_panel_output out)
{
    palTogglePad(output_pins[out].port, output_pins[out].pin);
}
