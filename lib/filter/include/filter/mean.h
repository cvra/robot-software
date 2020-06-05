#ifndef FILTER_MEAN_H
#define FILTER_MEAN_H

#ifdef __cplusplus
extern "C" {
#endif

float mean(float* set, size_t n);
int mean_int16(int16_t* set, size_t n);

#ifdef __cplusplus
}
#endif

#endif
