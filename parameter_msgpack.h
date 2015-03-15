#ifndef PARAMETER_MSGPACK_H
#define PARAMETER_MSGPACK_H

#include "parameter.h"

#ifdef __cplusplus
extern "C" {
#endif

#include <cmp/cmp.h>

typedef void (*parameter_msgpack_err_cb)(void *arg, const char *id, const char *err);

int parameter_msgpack_read_cmp(parameter_namespace_t *ns,
                               cmp_ctx_t *cmp,
                               parameter_msgpack_err_cb err_cb,
                               void *err_arg);

int parameter_msgpack_read(parameter_namespace_t *ns,
                           const char *buf,
                           size_t size,
                           parameter_msgpack_err_cb err_cb,
                           void *err_arg);

#ifdef __cplusplus
}
#endif

#endif /* PARAMETER_MSGPACK_H */