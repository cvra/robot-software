#ifndef CONTROL_PANEL_H
#define CONTROL_PANEL_H

enum control_panel_input {
    BUTTON_YELLOW = 0,
    BUTTON_GREEN,
    STARTER
};

enum control_panel_output {
    LED_READY = 0,
    LED_DEBUG,
    LED_ERROR,
    LED_POWER,
    LED_PC,
    LED_BUS,
    LED_YELLOW,
    LED_GREEN
};

#ifdef __cplusplus
extern "C" {
#endif

void control_panel_init(bool is_active_high);
bool control_panel_read(enum control_panel_input in);
void control_panel_set(enum control_panel_output out);
void control_panel_clear(enum control_panel_output out);
void control_panel_toggle(enum control_panel_output out);
bool control_panel_button_is_pressed(enum control_panel_input in);

#ifdef __cplusplus
}
#endif

#endif /* CONTROL_PANEL_H */
