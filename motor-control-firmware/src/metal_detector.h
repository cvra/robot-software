#ifndef METAL_DETECTOR_H
#define METAL_DETECTOR_H

#include <ch.h>

#ifdef __cplusplus
extern "C" {
#endif

void metal_detector_set_adc_samples(adcsample_t* samples, size_t n);

#ifdef __cplusplus
}
#endif

#endif /* METAL_DETECTOR_H*/
