#ifndef ENCODER_H
#define ENCODER_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

int32_t encoder_get_right(void);
int32_t encoder_get_left(void);
int32_t encoder_tick_diff(uint32_t enc_old, uint32_t enc_new) void encoder_init(void)

#ifdef __cplusplus
}
#endif

#endif /* ENCODER_H */
