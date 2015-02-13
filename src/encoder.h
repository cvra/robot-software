#ifndef ENCODER_H
#define ENCODER_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

uint32_t encoder_get_primary(void);
uint32_t encoder_get_secondary(void);
void encoder_init_primary(void);
void encoder_init_secondary(void);

#ifdef __cplusplus
}
#endif

#endif /* ENCODER_H */