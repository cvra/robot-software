#ifndef PARAMETER_PRINT_H
#define PARAMETER_PRINT_H

#include "parameter.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef int (*parameter_printfn_t)(void*, const char *, ... );

// print the parameter tree of the given namespace as yaml
void parameter_print(parameter_namespace_t *ns,
                     parameter_printfn_t printfn,
                     void *printfn_arg);

#ifdef __cplusplus
}
#endif

#endif /* PARAMETER_PRINT_H */