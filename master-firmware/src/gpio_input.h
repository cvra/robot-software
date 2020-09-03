#pragma once

#include <gpiod.h>
#include <absl/types/optional.h>

/** Wrapper around libgpiod, used to read a single GPIO at a time. */
class GpioInput {
private:
    struct gpiod_chip* chip;
    struct gpiod_line* line;

public:
    // Returns the state of the GPIO
    bool read();

    // Function used to open the GPIO for reading. Returns an empty object in
    // case it could not be opened.
    static absl::optional<GpioInput> open(std::string chipname, int line_num);

    GpioInput()
        : chip(nullptr)
        , line(nullptr)
    {
    }

    // The copy constructor and operator must be deleted, as handles to GPIOs
    // cannot be copied (they need to be re-opened separately).
    GpioInput(const GpioInput&) = delete; // cannot be copied
    GpioInput& operator=(const GpioInput& other) = delete;

    GpioInput(GpioInput&& other)
        : chip(other.chip)
        , line(other.line)
    {
        chip = other.chip;
        line = other.line;
    }

    GpioInput& operator=(GpioInput&& other)
    {
        std::swap(other.chip, chip);
        std::swap(other.line, line);
        return *this;
    }

    ~GpioInput();
};
