#include "stream.h"
#include <math.h>


void stream_set_prescaler(stream_config_t *stream_config, float frequency, float spin_frequency)
{
    stream_config->prescaler = nearbyint(spin_frequency / frequency);
    stream_config->counter = stream_config->prescaler;
}

void stream_enable(stream_config_t *stream_config, bool enabled)
{
    stream_config->enabled = enabled;
}

bool stream_update(stream_config_t *stream_config)
{
    if (stream_config->enabled) {
        stream_config->counter--;
        if (stream_config->counter == 0) {
            stream_config->counter = stream_config->prescaler;
            return true;
        } else {
            return false;
        }
    }

    return false;
}
