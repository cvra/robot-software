#include "parameter.h"
#include "parameter_print.h"

#define MALLOC_FAILED_WARNING "# warning: malloc failed during serialization!"


static void indent(int indent_lvl,
                   parameter_printfn_t printfn,
                   void *printfn_arg)
{
    int i;
    for (i = 0; i < indent_lvl; i++) {
        printfn(printfn_arg, "  ");
    }
}


static void print_array(float *a,
                        int len,
                        parameter_printfn_t printfn,
                        void *printfn_arg)
{
    printfn(printfn_arg, "[");
    int i;
    for (i = 0; i < len; i++) {
        if (i == len - 1) {
            printfn(printfn_arg, "%f", a[i]);
        } else {
            printfn(printfn_arg, "%f, ", a[i]);
        }
    }
    printfn(printfn_arg, "]\n");
}


static void param_entry_print(parameter_t *p,
                              int indent_lvl,
                              parameter_printfn_t printfn,
                              void *printfn_arg)
{
    indent(indent_lvl, printfn, printfn_arg);
    printfn(printfn_arg, "%s: ", p->id);
    if (!parameter_defined(p)) {
        printfn(printfn_arg, "~\n");
        return;
    }
    switch (p->type) {
    case _PARAM_TYPE_SCALAR: {
        printfn(printfn_arg, "%f\n", parameter_scalar_read(p));
        break;
    }
    case _PARAM_TYPE_INTEGER: {
        printfn(printfn_arg, "%d\n", parameter_integer_read(p));
        break;
    }
    case _PARAM_TYPE_BOOLEAN: {
        printfn(printfn_arg, "%s\n", parameter_boolean_read(p) ? "true":"false");
        break;
    }
    case _PARAM_TYPE_STRING: {
        int len = parameter_string_max_len(p);
        char *s = malloc(len);
        if (s) {
            parameter_string_read(p, s, len);
            printfn(printfn_arg, "\"%s\"\n", s);
            free(s);
        } else {
            printfn(printfn_arg, "\"\" %s\n", MALLOC_FAILED_WARNING);
        }
        break;
    }
    case _PARAM_TYPE_VECTOR: {
        int dim = parameter_vector_dim(p);
        float *v = malloc(dim * sizeof(float));
        if (v) {
            parameter_vector_read(p, v);
            print_array(v, dim, printfn, printfn_arg);
            free(v);
        } else {
            printfn(printfn_arg, "[] %s\n", MALLOC_FAILED_WARNING);
        }
        break;
    }
    case _PARAM_TYPE_VAR_VECTOR: {
        int dim = parameter_variable_vector_max_dim(p);
        float *v = malloc(dim * sizeof(float));
        if (v) {
            dim = parameter_variable_vector_read(p, v);
            print_array(v, dim, printfn, printfn_arg);
            free(v);
        } else {
            printfn(printfn_arg, "[] %s\n", MALLOC_FAILED_WARNING);
        }
        break;
    }
    }
}


static void param_ns_print(parameter_namespace_t *ns,
                           int indent_lvl,
                           parameter_printfn_t printfn,
                           void *printfn_arg)
{
    parameter_t *p = ns->parameter_list;
    while (p != NULL) {
        param_entry_print(p, indent_lvl, printfn, printfn_arg);
        p = p->next;
    }
    parameter_namespace_t *n = ns->subspaces;
    while (n != NULL) {
        indent(indent_lvl, printfn, printfn_arg);
        printfn(printfn_arg, "%s:\n", n->id);
        param_ns_print(n, indent_lvl + 1, printfn, printfn_arg);
        n = n->next;
    }
}


void parameter_print(parameter_namespace_t *ns,
                     parameter_printfn_t printfn,
                     void *printfn_arg)
{
    param_ns_print(ns, 0, printfn, printfn_arg);
}
