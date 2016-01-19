#include <string.h>
#include "parameter.h"
#include <parameter_port.h>

/*
 * Synchronization is implemented using the PARAMETER_LOCK() and
 * PARAMETER_UNLOCK() macros provided in the parameter_port.h.
 * Synchronization is required in the following places:
 *  - linked-list head pointer of the parameter list in a namespace
 *  - linked-list head pointer of the sub-namespace list in a namespace
 *  - the changed count of a namespace
 *  - the changed flag of a parameter
 *  - the parameter value
 * All other values are read-only once the parameter/namespace has been linked
 * into the parameter tree and therefore require no synchronization.
 * Access to the above values can be synchronized independently except for the
 * update of the changed flag, which has to be checked inside the critical
 * section to know whether the changed count has to be updated for the
 * containing namespaces. (The thread that succeeds in modifying the flag is
 * responsible for updating the counters. Note that the counters could
 * temporarily become negative in case the increment is interrupted by a
 * get_parameter() which decreases the counter. This poses no problem since the
 * check for the changed count correctly handles the signed integer counter)
 */


/* find the length of the next element in hierarchical id
 * returns number of characters until the first '/' or the entire
 * length if no '/' is found)
 */
static int id_split(const char *id, int id_len)
{
    int i = 0;
    for (i = 0; i < id_len; i++) {
        if (id[i] == '/') {
            return i;
        }
    }
    return id_len;
}


/*
 * get a sub-namespace of a namespace by id. search depth is only one level
 */
static parameter_namespace_t *get_subnamespace(parameter_namespace_t *ns,
                                               const char *ns_id,
                                               size_t ns_id_len)
{
    if (ns_id_len == 0) {
        return ns; // this allows to start with a '/' or have '//' instead of '/'
    }
    PARAMETER_LOCK();
    parameter_namespace_t *i = ns->subspaces;
    PARAMETER_UNLOCK();
    while (i != NULL) {
        if (strncmp(ns_id, i->id, ns_id_len) == 0 && i->id[ns_id_len] == '\0') {
            // if the first ns_id_len bytes of ns_id match with i->id and
            // i->id is only ns_id_len bytes long, we've found the namespace
            break;
        }
        i = i->next;
    }
    return i;
}

/*
 * get a parameter of a namespace by id. search depth is only one level
 */
static parameter_t *get_parameter(parameter_namespace_t *ns, const char *id,
                                  size_t param_id_len)
{
    if (param_id_len == 0) {
        return NULL;
    }
    PARAMETER_LOCK();
    parameter_t *i = ns->parameter_list;
    PARAMETER_UNLOCK();
    while (i != NULL) {
        if (strncmp(id, i->id, param_id_len) == 0 && i->id[param_id_len] == '\0') {
            // if the first param_id_len bytes of id match with i->id and
            // i->id is only param_id_len bytes long, we've found the parameter
            break;
        }
        i = i->next;
    }
    return i;
}

void parameter_namespace_declare(parameter_namespace_t *ns,
                                 parameter_namespace_t *parent,
                                 const char *id)
{
    ns->id = id;
    ns->changed_cnt = 0;
    ns->parent = parent;
    ns->subspaces = NULL;
    ns->parameter_list = NULL;
    if (parent != NULL) {
        PARAMETER_LOCK();
        // link into parent namespace
        ns->next = ns->parent->subspaces;
        ns->parent->subspaces = ns;
        PARAMETER_UNLOCK();
    } else {
        ns->next = NULL;
    }
}


parameter_namespace_t *_parameter_namespace_find_w_id_len(parameter_namespace_t *ns,
                                                const char *id, size_t id_len)
{
    parameter_namespace_t *nret = ns;
    uint32_t i = 0;
    while(nret != NULL && i < id_len) {
        int id_elem_len = id_split(&id[i], id_len - i);
        nret = get_subnamespace(nret, &id[i], id_elem_len);
        i += id_elem_len + 1;
    }
    return nret;
}

parameter_namespace_t *parameter_namespace_find(parameter_namespace_t *ns,
                                                const char *id)
{
    return _parameter_namespace_find_w_id_len(ns, id, strlen(id));
}

parameter_t *_parameter_find_w_id_len(parameter_namespace_t *ns,
                                      const char *id, size_t id_len)
{
    parameter_namespace_t *pns = ns;
    uint32_t i = 0;
    while(pns != NULL) {
        int id_elem_len = id_split(&id[i], id_len - i);
        if (id_elem_len + i < id_len) {
            pns = get_subnamespace(pns, &id[i], id_elem_len);
        } else {
            return get_parameter(pns, &id[i], id_elem_len);
        }
        i += id_elem_len + 1;
    }
    return NULL;
}

parameter_t *parameter_find(parameter_namespace_t *ns, const char *id)
{
    return _parameter_find_w_id_len(ns, id, strlen(id));
}

void _parameter_declare(parameter_t *p, parameter_namespace_t *ns,
                        const char *id)
{
    p->id = id;
    p->ns = ns;
    p->changed = false;
    p->defined = false;
    PARAMETER_LOCK();
    // link into namespace
    p->next = p->ns->parameter_list;
    p->ns->parameter_list = p;
    PARAMETER_UNLOCK();
}

bool parameter_namespace_contains_changed(const parameter_namespace_t *ns)
{
    PARAMETER_LOCK();
    uint32_t changed_cnt = ns->changed_cnt;
    PARAMETER_UNLOCK();
    return (changed_cnt > 0);
}

bool parameter_changed(const parameter_t *p)
{
    PARAMETER_LOCK();
    bool changed = p->changed;
    PARAMETER_UNLOCK();
    return changed;
}

bool parameter_defined(const parameter_t *p)
{
    if (p == NULL) {
        return false;
    }
    PARAMETER_LOCK();
    bool defined = p->defined;
    PARAMETER_UNLOCK();
    return defined;
}

void _parameter_changed_set(parameter_t *p)
{
    PARAMETER_LOCK();
    bool changed_was_set = p->changed;
    p->changed = true;
    PARAMETER_UNLOCK();
    if (changed_was_set) {
        return;
    }
    // if the above "compare and set" passes, the changed count can safely
    // be incremented for the namespaces
    parameter_namespace_t *ns = p->ns;
    while (ns != NULL) {
        PARAMETER_LOCK();
        ns->changed_cnt++;
        PARAMETER_UNLOCK();
        ns = ns->parent;
    }
    p->defined = true;
}

void _parameter_changed_clear(parameter_t *p)
{
    PARAMETER_LOCK();
    bool changed_was_set = p->changed;
    p->changed = false;
    PARAMETER_UNLOCK();
    if (!changed_was_set) {
        return;
    }
    // if the above "compare and set" passes, the changed count can safely
    // be decremented for the namespaces
    parameter_namespace_t *ns = p->ns;
    while (ns != NULL) {
        PARAMETER_LOCK();
        ns->changed_cnt--; // here change counts can temporarily become negative
        PARAMETER_UNLOCK();
        ns = ns->parent;
    }
}

/*
 * Scalar type parameter
 */

void parameter_scalar_declare(parameter_t *p, parameter_namespace_t *ns,
                              const char *id)
{
    p->type = _PARAM_TYPE_SCALAR;
    _parameter_declare(p, ns, id);
}

void parameter_scalar_declare_with_default(parameter_t *p,
                                           parameter_namespace_t *ns,
                                           const char *id,
                                           float default_val)
{
    p->value.s = default_val;
    p->type = _PARAM_TYPE_SCALAR;
    _parameter_declare(p, ns, id);
    _parameter_changed_set(p);
}

float parameter_scalar_get(parameter_t *p)
{
    _parameter_changed_clear(p);
    return parameter_scalar_read(p);
}

float parameter_scalar_read(parameter_t *p)
{
    PARAMETER_ASSERT(p->type == _PARAM_TYPE_SCALAR);
    PARAMETER_LOCK();
    PARAMETER_ASSERT(p->defined == true);
    float ret = p->value.s;
    PARAMETER_UNLOCK();
    return ret;
}

void parameter_scalar_set(parameter_t *p, float value)
{
    PARAMETER_ASSERT(p->type == _PARAM_TYPE_SCALAR);
    PARAMETER_LOCK();
    p->value.s = value;
    PARAMETER_UNLOCK();
    _parameter_changed_set(p);
}


/*
 * Integer type parameter
 */

void parameter_integer_declare(parameter_t *p, parameter_namespace_t *ns,
                              const char *id)
{
    p->type = _PARAM_TYPE_INTEGER;
    _parameter_declare(p, ns, id);
}

void parameter_integer_declare_with_default(parameter_t *p,
                                           parameter_namespace_t *ns,
                                           const char *id,
                                           int32_t default_val)
{
    p->value.i = default_val;
    p->type = _PARAM_TYPE_INTEGER;
    _parameter_declare(p, ns, id);
    _parameter_changed_set(p);
}

int32_t parameter_integer_get(parameter_t *p)
{
    _parameter_changed_clear(p);
    return parameter_integer_read(p);
}

int32_t parameter_integer_read(parameter_t *p)
{
    PARAMETER_ASSERT(p->type == _PARAM_TYPE_INTEGER);
    PARAMETER_LOCK();
    PARAMETER_ASSERT(p->defined == true);
    int32_t ret = p->value.i;
    PARAMETER_UNLOCK();
    return ret;
}

void parameter_integer_set(parameter_t *p, int32_t value)
{
    PARAMETER_ASSERT(p->type == _PARAM_TYPE_INTEGER);
    PARAMETER_LOCK();
    p->value.i = value;
    PARAMETER_UNLOCK();
    _parameter_changed_set(p);
}

/* Boolean type parameter. */

void parameter_boolean_declare(parameter_t *p, parameter_namespace_t *ns, const char *id)
{
    p->type = _PARAM_TYPE_BOOLEAN;
    _parameter_declare(p, ns, id);
}

void parameter_boolean_declare_with_default(parameter_t *p,
                                            parameter_namespace_t *ns,
                                            const char *id,
                                            bool default_val)
{
    p->value.b = default_val;
    p->type = _PARAM_TYPE_BOOLEAN;
    _parameter_declare(p, ns, id);
    _parameter_changed_set(p);
}

void parameter_boolean_set(parameter_t *p, bool value)
{
    PARAMETER_ASSERT(p->type == _PARAM_TYPE_BOOLEAN);

    PARAMETER_LOCK();
    p->value.b = value;
    PARAMETER_UNLOCK();
    _parameter_changed_set(p);
}

bool parameter_boolean_get(parameter_t *p)
{
    _parameter_changed_clear(p);
    return parameter_boolean_read(p);
}

bool parameter_boolean_read(parameter_t *p)
{
    PARAMETER_ASSERT(p->type == _PARAM_TYPE_BOOLEAN);
    PARAMETER_LOCK();
    PARAMETER_ASSERT(p->defined == true);
    bool ret = p->value.b;
    PARAMETER_UNLOCK();
    return ret;
}


/*
 * Vector type parameter
 */

void parameter_vector_declare(parameter_t *p, parameter_namespace_t *ns,
                              const char *id, float *buf, uint16_t dim)
{
    p->type = _PARAM_TYPE_VECTOR;
    p->value.vect.buf = buf;
    p->value.vect.dim = dim;
    _parameter_declare(p, ns, id);
}

void parameter_vector_declare_with_default(parameter_t *p,
                                           parameter_namespace_t *ns,
                                           const char *id,
                                           float *buf,
                                           uint16_t dim)
{
    parameter_vector_declare(p, ns, id, buf, dim);
    _parameter_changed_set(p);
}

uint16_t parameter_vector_dim(parameter_t *p)
{
    PARAMETER_ASSERT(p->type == _PARAM_TYPE_VECTOR);
    return p->value.vect.dim;
}

void parameter_vector_get(parameter_t *p, float *out)
{
    _parameter_changed_clear(p);
    parameter_vector_read(p, out);
}

void parameter_vector_read(parameter_t *p, float *out)
{
    PARAMETER_ASSERT(p->type == _PARAM_TYPE_VECTOR);
    PARAMETER_LOCK();
    PARAMETER_ASSERT(p->defined == true);
    int i;
    for (i = 0; i < p->value.vect.dim; i++) {
        out[i] = p->value.vect.buf[i];
    }
    PARAMETER_UNLOCK();
}

void parameter_vector_set(parameter_t *p, const float *v)
{
    PARAMETER_ASSERT(p->type == _PARAM_TYPE_VECTOR);
    PARAMETER_LOCK();
    int i;
    for (i = 0; i < p->value.vect.dim; i++) {
        p->value.vect.buf[i] = v[i];
    }
    PARAMETER_UNLOCK();
    _parameter_changed_set(p);
}


/*
 * Variable size vector type parameter
 */

void parameter_variable_vector_declare(parameter_t *p,
                                       parameter_namespace_t *ns,
                                       const char *id,
                                       float *buf,
                                       uint16_t buf_size)
{
    p->type = _PARAM_TYPE_VAR_VECTOR;
    p->value.vect.buf = buf;
    p->value.vect.buf_dim = buf_size;
    _parameter_declare(p, ns, id);
}

void parameter_variable_vector_declare_with_default(parameter_t *p,
                                                    parameter_namespace_t *ns,
                                                    const char *id,
                                                    float *buf,
                                                    uint16_t buf_size,
                                                    uint16_t init_size)
{
    parameter_variable_vector_declare(p, ns, id , buf, buf_size);
    PARAMETER_ASSERT(init_size <= buf_size);
    p->value.vect.dim = init_size;
    _parameter_changed_set(p);
}

uint16_t parameter_variable_vector_max_dim(parameter_t *p)
{
    PARAMETER_ASSERT(p->type == _PARAM_TYPE_VAR_VECTOR);
    return p->value.vect.buf_dim;
}

uint16_t parameter_variable_vector_get(parameter_t *p, float *out)
{
    _parameter_changed_clear(p);
    return parameter_variable_vector_read(p, out);
}

uint16_t parameter_variable_vector_read(parameter_t *p, float *out)
{
    PARAMETER_ASSERT(p->type == _PARAM_TYPE_VAR_VECTOR);
    PARAMETER_LOCK();
    PARAMETER_ASSERT(p->defined == true);
    int i;
    for (i = 0; i < p->value.vect.dim; i++) {
        out[i] = p->value.vect.buf[i];
    }
    uint16_t ret = p->value.vect.dim;
    PARAMETER_UNLOCK();
    return ret;
}

void parameter_variable_vector_set(parameter_t *p, const float *v, uint16_t dim)
{
    PARAMETER_ASSERT(p->type == _PARAM_TYPE_VAR_VECTOR);
    PARAMETER_LOCK();
    PARAMETER_ASSERT(dim <= p->value.vect.buf_dim);
    int i;
    for (i = 0; i < dim; i++) {
        p->value.vect.buf[i] = v[i];
    }
    p->value.vect.dim = dim;
    PARAMETER_UNLOCK();
    _parameter_changed_set(p);
}


/*
 * String type parameter
 */

void parameter_string_declare(parameter_t *p,
                              parameter_namespace_t *ns,
                              const char *id,
                              char *buf,
                              uint16_t buf_size)
{
    p->type = _PARAM_TYPE_STRING;
    p->value.str.buf = buf;
    p->value.str.buf_len = buf_size;
    _parameter_declare(p, ns, id);
}

void parameter_string_declare_with_default(parameter_t *p,
                                           parameter_namespace_t *ns,
                                           const char *id,
                                           char *buf,
                                           uint16_t buf_size,
                                           const char *default_str)
{
    parameter_string_declare(p, ns, id , buf, buf_size);
    p->value.str.len = strlen(default_str);
    PARAMETER_ASSERT(p->value.str.len <= buf_size);
    strncpy(p->value.str.buf, default_str, buf_size);
    _parameter_changed_set(p);
}

uint16_t parameter_string_max_len(parameter_t *p)
{
    PARAMETER_ASSERT(p->type == _PARAM_TYPE_STRING);
    return p->value.str.buf_len + 1; // +1 for '\0' terminator
}

uint16_t parameter_string_get(parameter_t *p, char *out, uint16_t out_size)
{
    _parameter_changed_clear(p);
    return parameter_string_read(p, out, out_size);
}

uint16_t parameter_string_read(parameter_t *p, char *out, uint16_t out_size)
{
    PARAMETER_ASSERT(p->type == _PARAM_TYPE_STRING);
    PARAMETER_LOCK();
    PARAMETER_ASSERT(p->defined == true);
    uint16_t len = p->value.str.len;
    if (out_size > len) {
        memcpy(out, p->value.str.buf, len);
        out[len] = '\0';
    } else {
        memcpy(out, p->value.str.buf, out_size-1);
        out[out_size-1] = '\0';
    }
    PARAMETER_UNLOCK();
    return len;
}

void parameter_string_set(parameter_t *p, const char *str)
{
    parameter_string_set_w_len(p, str, strlen(str));
}

void parameter_string_set_w_len(parameter_t *p, const char *str, uint16_t len)
{
    PARAMETER_ASSERT(p->type == _PARAM_TYPE_STRING);
    PARAMETER_LOCK();
    PARAMETER_ASSERT(len <= p->value.str.buf_len);
    memcpy(p->value.str.buf, str, len);
    p->value.str.len = len;
    PARAMETER_UNLOCK();
    _parameter_changed_set(p);
}
