#include "gpio_input.h"
#include <error/error.h>

GpioInput::~GpioInput()
{
    if (line) {
        DEBUG("releasing line");
        gpiod_line_release(line);
    }

    if (chip) {
        DEBUG("closing chip");
        gpiod_chip_close(chip);
    }
}

bool GpioInput::read()
{
    if (!line) {
        return false;
    }
    return gpiod_line_get_value(line) == 1;
}

absl::optional<GpioInput> GpioInput::open(std::string chipname, int line_num)
{
    GpioInput result;
    result.chip = gpiod_chip_open_by_name(chipname.c_str());
    if (!result.chip) {
        WARNING("Could not open gpio chip %s", chipname.c_str());
        return {};
    }

    result.line = gpiod_chip_get_line(result.chip, line_num);

    if (!result.line) {
        WARNING("Could not open gpio line");
        return {};
    }

    int ret = gpiod_line_request_input(result.line, "master-firmware");

    if (ret < 0) {
        WARNING("Request line as input failed");
        return {};
    }

    return {std::move(result)};
}
