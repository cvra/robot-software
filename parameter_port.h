#ifndef PARAMETER_PORT_H
#define PARAMETER_PORT_H

#ifdef __cplusplus
extern "C" {
#endif

#define PARAMETER_LOCK() {}

#define PARAMETER_UNLOCK() {}

#define PARAMETER_ASSERT(check) {}

// an allocated buffer is needed to convert message pack objects
// only one allocation is made at any moment which allows the use of a
// static buffer.
// the maximum allocated size is the maximum of parameter id/namespace string,
// the largest parameter array, matrix or string parameter
#define PARAMETER_MSGPACK_MALLOC(size) malloc(size)
#define PARAMETER_MSGPACK_FREE(ptr) free(ptr)

#ifdef __cplusplus
}
#endif

#endif /* PARAMETER_PORT_H */

