#ifndef INDEX_H
#define INDEX_H

#include <ch.h>

#ifdef __cplusplus
extern "C" {
#endif


void index_init(void);
void index_get_position(float *out_position, uint32_t *out_update_count);

#ifdef __cplusplus
}
#endif

#endif /* INDEX_H */
