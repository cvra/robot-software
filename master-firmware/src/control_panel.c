#include <error/error.h>
#include "control_panel.h"

const char* control_panel_input[] = {
    "BUTTON_YELLOW",
    "BUTTON_GREEN",
    "STARTER",
};

const char* control_panel_output[] = {
    "LED_READY",
    "LED_DEBUG",
    "LED_ERROR",
    "LED_POWER",
    "LED_PC",
    "LED_BUS",
    "LED_YELLOW",
    "LED_GREEN",
};

static int out_status[LED_GREEN + 1] = {0};

void control_panel_init(bool is_active_high)
{
    // TODO(antoinealb): Open all the files in sysfs and store handles to them
    (void)is_active_high;
}

bool control_panel_read(enum control_panel_input in)
{
    WARNING_EVERY_N(100, "%s(%s) not implemented yet.", __FUNCTION__, control_panel_input[in]);
    return 0;
}

bool control_panel_button_is_pressed(enum control_panel_input in)
{
    WARNING_EVERY_N(100, "%s(%s) not implemented yet.", __FUNCTION__, control_panel_input[in]);
    return 0;
}

void control_panel_set(enum control_panel_output out)
{
    if (out_status[out] == 1) {
        return;
    }
    out_status[out] = 1;

    WARNING("%s(%s) not implemented yet.", __FUNCTION__, control_panel_output[out]);
}

void control_panel_clear(enum control_panel_output out)
{
    if (out_status[out] == 0) {
        return;
    }
    out_status[out] = 0;

    WARNING("%s(%s) not implemented yet.", __FUNCTION__, control_panel_output[out]);
}

void control_panel_toggle(enum control_panel_output out)
{
    out_status[out] = 1 - out_status[out];
    WARNING("%s(%s) not implemented yet.", __FUNCTION__, control_panel_output[out]);
}
