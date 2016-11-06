#ifndef STREAM_H
#define STREAM_H


#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    bool enabled;
    uint16_t prescaler;
    uint16_t counter;
} stream_config_t;


void stream_set_prescaler(stream_config_t *stream_config, float frequency, float spin_frequency);
void stream_enable(stream_config_t *stream_config, bool enabled);
bool stream_update(stream_config_t *stream_config);


#ifdef __cplusplus
}
#endif

#endif /* STREAM_H */
