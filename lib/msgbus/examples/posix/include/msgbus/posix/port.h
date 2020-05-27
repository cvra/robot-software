#ifndef PORT_H
#define PORT_H
#include <pthread.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    pthread_mutex_t mutex;
    pthread_cond_t cond;
} condvar_wrapper_t;

#ifdef __cplusplus
}
#endif
#endif
