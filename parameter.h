#ifndef PARAMETER_H
#define PARAMETER_H

#include <stdint.h>
#include <stdlib.h>


typedef struct parameter_namespace_s parameter_namespace_t;
typedef struct parameter_s parameter_t;

struct parameter_namespace_s {
    const char *id;
    uint32_t changed_cnt;
    parameter_namespace_t *parent;
    parameter_namespace_t *subspaces;
    parameter_namespace_t *next;
    parameter_t *parameter_list;
};

struct _param_val_str_s {
    char *buf;
    uint16_t size;
};

struct _param_val_vector_s {
    float *buf;
    uint16_t len;
};

struct _param_val_matrix_s {
    float *buf;
    uint16_t n;
    uint16_t m;
};

#define _PARAM_TYPE_BOOL        1
#define _PARAM_TYPE_SCALAR      2
#define _PARAM_TYPE_INTEGER     3
#define _PARAM_TYPE_STRING      4
#define _PARAM_TYPE_VECTOR      5
#define _PARAM_TYPE_MATRIX      6

struct parameter_s {
    const char *id;
    parameter_namespace_t *ns;
    parameter_t *next;
    bool changed;
    bool set;
    uint8_t type;
    union {
        bool b;
        float s;
        int32_t i;
        struct _param_val_str_s str;
        struct _param_val_vector_s vect;
        struct _param_val_matrix_s mat;
    } value;
};


#ifdef __cplusplus
extern "C" {
#endif


void parameter_namespace_declare(parameter_namespace_t *ns,
                                 parameter_namespace_t *parent,
                                 const char *id);

parameter_namespace_t *parameter_namespace_find(parameter_namespace_t *ns,
                                                const char *id);
/* [internal API]
 * id is a char buffer with explicit length instead of a null terminated string.
 */
parameter_namespace_t *_parameter_namespace_find_w_id_len(parameter_namespace_t *ns,
                                                const char *id, size_t id_len);

bool parameter_namespace_contains_changed(const parameter_namespace_t *ns);

/*
 * Get the parameter by id.
 * The id is relative to the namespace.
 * Returns NULL if the parameter doesn't exist
 */
parameter_t *parameter_find(const parameter_namespace_t *ns, const char *id);
/*
 * [internal API]
 * id is a char buffer with explicit length instead of a null terminated string.
 */
parameter_t *_parameter_find_w_id_len(const parameter_namespace_t *ns,
                                      const char *id, size_t id_len);

/*
 * Test whether the parameter has been set.
 * Returns true if the parameter has been set, false otherwise.
 * Returns false if p if a NULL pointer.
 */
bool parameter_defined(const parameter_t *p);

/*
 * Test whether the parameter was updated.
 * Returns true if the parameter has been updated since the last get call for
 * this parameter, false otherwise.
 * Returns false if p is a NULL pointer.
 */
bool parameter_changed(const parameter_t *p);



/* [internal API]
 * initialize the parameter without the type specific fields.
 */
void _parameter_declare(parameter_t *p, parameter_namespace_t *ns,
                        const char *id);


/*
 * Scalar type parameter
 */

void parameter_scalar_declare(parameter_t *p, parameter_namespace_t *ns,
                              const char *id);

void parameter_scalar_declare_with_default(parameter_t *p,
                                           parameter_namespace_t *ns,
                                           const char *id,
                                           float default_val);
float parameter_scalar_get(parameter_t *p);
float parameter_scalar_read(parameter_t *p);
void parameter_scalar_set(parameter_t *p, float value);


#ifdef __cplusplus
}
#endif

#endif /* PARAMETER_H */
