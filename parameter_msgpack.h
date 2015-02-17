#ifndef PARAMETER_MSGPACK_H
#define PARAMETER_MSGPACK_H

#include "parameter.h"

#ifdef __cplusplus
extern "C" {
#endif

int parameter_msgpack_read(parameter_namespace_t *ns, const char *buf, size_t size);

#ifdef __cplusplus
}
#endif

#endif /* PARAMETER_MSGPACK_H */