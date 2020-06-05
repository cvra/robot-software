#include <absl/flags/flag.h>
#include <error/error.h>
#include "control_panel.h"
#include <unordered_map>
#include <string>
#include <cstdio>
#include <cerrno>

ABSL_FLAG(std::string, led_ready_path, "/sys/class/leds/ready", "Path in sysfs used to control LED_READY.");
ABSL_FLAG(std::string, led_debug_path, "/sys/class/leds/debug", "Path in sysfs used to control LED_DEBUG.");
ABSL_FLAG(std::string, led_error_path, "/sys/class/leds/error", "Path in sysfs used to control LED_ERROR.");
ABSL_FLAG(std::string, led_power_path, "/sys/class/leds/power", "Path in sysfs used to control LED_POWER.");
ABSL_FLAG(std::string, led_pc_path, "/sys/class/leds/pc", "Path in sysfs used to control LED_PC.");
ABSL_FLAG(std::string, led_bus_path, "/sys/class/leds/bus", "Path in sysfs used to control LED_BUS.");
// TODO: Support for RGB led

struct LedInfo {
    FILE* brightness_file = nullptr;
    int max_brightness = 0;
    bool out_status = false;
};

std::unordered_map<enum control_panel_output, LedInfo> led_infos;

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

static void set_output(enum control_panel_output out, bool value)
{
    auto led_info = led_infos.find(out);

    DEBUG("seting %s to %d", control_panel_output[out], value);

    if (led_info == led_infos.end()) {
        WARNING("can not set output %d", out);
        return;
    }

    std::string msg = std::to_string(value * led_info->second.max_brightness);
    fwrite(msg.c_str(), msg.length(), 1, led_info->second.brightness_file);
    fflush(led_info->second.brightness_file);
    DEBUG("writing '%s' to out %d", msg.c_str(), out);

    led_info->second.out_status = value;
}

static void open_led(enum control_panel_output led_num, std::string path)
{
    FILE* f;
    LedInfo led_info;

    DEBUG("output %d is at %s", led_num, path.c_str());

    // First, get the maximum brightness level for this LED
    std::string max_brightness_path = path + "/max_brightness";
    f = fopen(max_brightness_path.c_str(), "r");
    if (f == NULL) {
        WARNING("could not open '%s' for led %d: %s", max_brightness_path.c_str(), led_num, strerror(errno));
        return;
    }

    fscanf(f, "%d", &led_info.max_brightness);

    // Then, open the control file for this LED
    std::string brightness_path = path + "/brightness";
    led_info.brightness_file = fopen(brightness_path.c_str(), "w");

    if (led_info.brightness_file == nullptr) {
        WARNING("could not open '%s' for led %d: %s", brightness_path.c_str(), led_num, strerror(errno));
        return;
    }

    // keep the file descriptor cached around
    led_infos[led_num] = led_info;

    // Turn off all LEDs by default
    set_output(led_num, false);

    DEBUG("opened %s succesfully", control_panel_output[led_num]);
}

void control_panel_init(bool is_active_high)
{
    (void)is_active_high;
    open_led(LED_READY, absl::GetFlag(FLAGS_led_ready_path));
    open_led(LED_DEBUG, absl::GetFlag(FLAGS_led_debug_path));
    open_led(LED_ERROR, absl::GetFlag(FLAGS_led_error_path));
    open_led(LED_POWER, absl::GetFlag(FLAGS_led_power_path));
    open_led(LED_PC, absl::GetFlag(FLAGS_led_pc_path));
    open_led(LED_BUS, absl::GetFlag(FLAGS_led_bus_path));
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
    set_output(out, true);
}

void control_panel_clear(enum control_panel_output out)
{
    set_output(out, false);
}

void control_panel_toggle(enum control_panel_output out)
{
    auto led = led_infos.find(out);
    if (led == led_infos.end()) {
        WARNING_EVERY_N(10, "could not read back state for %s", control_panel_output[out]);
        return;
    }

    set_output(out, !led->second.out_status);
}
