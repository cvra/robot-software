#ifndef FILTER_BASIC_H
#define FILTER_BASIC_H

#ifdef __cplusplus
extern "C" {
#endif

float filter_limit(float value, float min, float max);
float filter_limit_sym(float value, float limit);

#ifdef __cplusplus
}
#endif

#endif
