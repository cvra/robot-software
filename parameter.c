#include <string.h>
#include "parameter.h"
#include <parameter_port.h>



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
 * parameter types
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
#if PARAMETER_CHECKS_EN
    _parameter_assert(p->type == _PARAM_TYPE_SCALAR);
#endif
    _parameter_changed_clear(p);
    PARAMETER_LOCK();
    float ret = p->value.s;
    PARAMETER_UNLOCK();
    return ret;
}

float parameter_scalar_read(parameter_t *p)
{
#if PARAMETER_CHECKS_EN
    _parameter_assert(p->type == _PARAM_TYPE_SCALAR);
#endif
    PARAMETER_LOCK();
    float ret = p->value.s;
    PARAMETER_UNLOCK();
    return ret;
}

void parameter_scalar_set(parameter_t *p, float value)
{
#if PARAMETER_CHECKS_EN
    _parameter_assert(p->type == _PARAM_TYPE_SCALAR);
#endif
    PARAMETER_LOCK();
    p->value.s = value;
    PARAMETER_UNLOCK();
    _parameter_changed_set(p);
}
