#include <string.h>
#include <cmp_mem_access/cmp_mem_access.h>
#include <parameter_port.h>
#include "parameter_msgpack.h"


static int discard_msgpack_element(cmp_object_t *obj,
                                   cmp_ctx_t *cmp,
                                   parameter_msgpack_err_cb err_cb,
                                   void *err_arg,
                                   const char *err_id)
{
    (void)cmp; // ignore unused variable warning. todo
    switch (obj->type) {
    case CMP_TYPE_POSITIVE_FIXNUM:
    case CMP_TYPE_NIL:
    case CMP_TYPE_BOOLEAN:
    case CMP_TYPE_FLOAT:
    case CMP_TYPE_DOUBLE:
    case CMP_TYPE_UINT8:
    case CMP_TYPE_UINT16:
    case CMP_TYPE_UINT32:
    case CMP_TYPE_UINT64:
    case CMP_TYPE_SINT8:
    case CMP_TYPE_SINT16:
    case CMP_TYPE_SINT32:
    case CMP_TYPE_SINT64:
    case CMP_TYPE_NEGATIVE_FIXNUM:
        return 0;
    default:
        // todo discarding namespaces / vectors won't work
        err_cb(err_arg, err_id, "discarding failed");
        return -1;
  }
}


static bool get_float(cmp_object_t *obj, float *out)
{
    float valf;
    int64_t vali;
    if (cmp_object_as_float(obj, &valf)) {
        *out = valf;
        return true;
    } else if (cmp_object_as_sinteger(obj, &vali)) {
        *out = vali;
        return true;
    }
    return false;
}

static bool get_int(cmp_object_t *obj, int32_t *out)
{
    if (cmp_object_as_int(obj, out)) {
        return true;
    }
    return false;
}

static bool get_bool(cmp_object_t *obj, bool *out)
{
    if (cmp_object_as_bool(obj, out)) {
        return true;
    }
    return false;
}

static int get_vector(cmp_ctx_t *cmp,
                      float *buf,
                      int len,
                      parameter_msgpack_err_cb err_cb,
                      void *err_arg,
                      const char *err_id)
{
    bool err = false;
    int i;
    for (i = 0; i < len; i++) {
        cmp_object_t obj;
        if (!cmp_read_object(cmp, &obj)) {
            err_cb(err_arg, err_id, "could not read object");
            return -1;
        }
        float v;
        if (!err && get_float(&obj, &v)) {
            buf[i] = v;
        } else {
            if (!err) {
                err = true;
                err_cb(err_arg, err_id, "warning: type mismatch");
            }
            int ret = discard_msgpack_element(&obj, cmp, err_cb, err_arg, err_id);
            if (ret != 0) {
                return ret;
            }
        }
    }
    if (err) {
        return -1;
    } else {
        return 0;
    }
}

static int read_parameter_scalar(parameter_t *p,
                                 cmp_object_t *obj,
                                 cmp_ctx_t *cmp,
                                 parameter_msgpack_err_cb err_cb,
                                 void *err_arg)
{
    float v;
    if (get_float(obj, &v)) {
        parameter_scalar_set(p, v);
    } else {
        err_cb(err_arg, p->id, "warning: type mismatch");
        int ret = discard_msgpack_element(obj, cmp, err_cb, err_arg, p->id);
        if (ret != 0) {
            return ret;
        }
    }
    return 0;
}

static int read_parameter_integer(parameter_t *p,
                                  cmp_object_t *obj,
                                  cmp_ctx_t *cmp,
                                  parameter_msgpack_err_cb err_cb,
                                  void *err_arg)
{
    int32_t v;
    if (get_int(obj, &v)) {
        parameter_integer_set(p, v);
    } else {
        err_cb(err_arg, p->id, "warning: type mismatch");
        int ret = discard_msgpack_element(obj, cmp, err_cb, err_arg, p->id);
        if (ret != 0) {
            return ret;
        }
    }
    return 0;
}

static bool read_parameter_boolean(parameter_t *p,
                                   cmp_object_t *obj,
                                   cmp_ctx_t *cmp,
                                   parameter_msgpack_err_cb err_cb,
                                   void *err_arg)
{
    bool v;
    if (get_bool(obj, &v)) {
        parameter_boolean_set(p, v);
    } else {
        err_cb(err_arg, p->id, "warning: type mismatch");
        int ret = discard_msgpack_element(obj, cmp, err_cb, err_arg, p->id);
        if (ret != 0) {
            return ret;
        }
    }
    return 0;
}

static int read_parameter_vector(parameter_t *p,
                                 cmp_object_t *obj,
                                 cmp_ctx_t *cmp,
                                 parameter_msgpack_err_cb err_cb,
                                 void *err_arg)
{
    uint32_t array_size;
    if (cmp_object_as_array(obj, &array_size)) {
        if (array_size != p->value.vect.dim) {
            err_cb(err_arg, p->id, "warning: wrong vector dimension");
            int ret = discard_msgpack_element(obj, cmp, err_cb, err_arg, p->id);
            if (ret != 0) {
                return ret;
            }
        }
        float *buf = PARAMETER_MSGPACK_MALLOC(array_size * sizeof(float));
        if (buf == NULL) {
            err_cb(err_arg, p->id, "warning: allocation failed");
            int ret = discard_msgpack_element(obj, cmp, err_cb, err_arg, p->id);
            if (ret != 0) {
                return ret;
            }
        }
        int ret = get_vector(cmp, buf, array_size, err_cb, err_arg, p->id);
        if (ret != 0) {
            PARAMETER_MSGPACK_FREE(buf);
            return ret;
        }
        parameter_vector_set(p, buf);
        PARAMETER_MSGPACK_FREE(buf);
    } else {
        err_cb(err_arg, p->id, "warning: type mismatch");
        int ret = discard_msgpack_element(obj, cmp, err_cb, err_arg, p->id);
        if (ret != 0) {
            return ret;
        }
    }
    return 0;
}

static int read_parameter_var_vector(parameter_t *p,
                                     cmp_object_t *obj,
                                     cmp_ctx_t *cmp,
                                     parameter_msgpack_err_cb err_cb,
                                     void *err_arg)
{
    uint32_t array_size;
    if (cmp_object_as_array(obj, &array_size)) {
        if (array_size > p->value.vect.buf_dim) {
            err_cb(err_arg, p->id, "warning: vector dimension too big");
            int ret = discard_msgpack_element(obj, cmp, err_cb, err_arg, p->id);
            if (ret != 0) {
                return ret;
            }
        }
        float *buf = PARAMETER_MSGPACK_MALLOC(array_size * sizeof(float));
        if (buf == NULL) {
            err_cb(err_arg, p->id, "warning: allocation failed");
            int ret = discard_msgpack_element(obj, cmp, err_cb, err_arg, p->id);
            if (ret != 0) {
                return ret;
            }
        }
        int ret = get_vector(cmp, buf, array_size, err_cb, err_arg, p->id);
        if (ret != 0) {
            PARAMETER_MSGPACK_FREE(buf);
            return ret;
        }
        parameter_variable_vector_set(p, buf, array_size);
        PARAMETER_MSGPACK_FREE(buf);
    } else {
        err_cb(err_arg, p->id, "warning: type mismatch");
        int ret = discard_msgpack_element(obj, cmp, err_cb, err_arg, p->id);
        if (ret != 0) {
            return ret;
        }
    }
    return 0;
}


static int read_parameter_string(parameter_t *p,
                                 cmp_object_t *obj,
                                 cmp_ctx_t *cmp,
                                 parameter_msgpack_err_cb err_cb,
                                 void *err_arg)
{
    uint32_t str_len;
    if (!cmp_object_as_str(obj, &str_len)) {
        err_cb(err_arg, p->id, "warning: type mismatch");
        return discard_msgpack_element(obj, cmp, err_cb, err_arg, p->id);
    }
    if (str_len >= p->value.str.buf_len) {
        err_cb(err_arg, p->id, "warning: string is too long");
        return discard_msgpack_element(obj, cmp, err_cb, err_arg, p->id);
    }
    char *str_buf = PARAMETER_MSGPACK_MALLOC(str_len);
    if (str_buf == NULL) {
        err_cb(err_arg, p->id, "warning: allocation failed");
        return discard_msgpack_element(obj, cmp, err_cb, err_arg, p->id);
    }
    if (!cmp->read(cmp, str_buf, str_len)) {
        err_cb(err_arg, p->id, "read error");
        PARAMETER_MSGPACK_FREE(str_buf);
        return -1;
    }
    parameter_string_set_w_len(p, str_buf, str_len);
    PARAMETER_MSGPACK_FREE(str_buf);
    return 0;
}

static int read_parameter(parameter_t *p,
                          cmp_object_t *obj,
                          cmp_ctx_t *cmp,
                          parameter_msgpack_err_cb err_cb,
                          void *err_arg)
{
    switch (p->type) {
    case _PARAM_TYPE_SCALAR:
        return read_parameter_scalar(p, obj, cmp, err_cb, err_arg);
    case _PARAM_TYPE_INTEGER:
        return read_parameter_integer(p, obj, cmp, err_cb, err_arg);
    case _PARAM_TYPE_BOOLEAN:
        return read_parameter_boolean(p, obj, cmp, err_cb, err_arg);
    case _PARAM_TYPE_VECTOR:
        return read_parameter_vector(p, obj, cmp, err_cb, err_arg);
    case _PARAM_TYPE_VAR_VECTOR:
        return read_parameter_var_vector(p, obj, cmp, err_cb, err_arg);
    case _PARAM_TYPE_STRING:
        return read_parameter_string(p, obj, cmp, err_cb, err_arg);
    default:
        err_cb(err_arg, p->id, "TODO not implemented yet");
        return discard_msgpack_element(obj, cmp, err_cb, err_arg, p->id);
    }
}


static int read_namespace(parameter_namespace_t *ns,
                          uint32_t map_size,
                          cmp_ctx_t *cmp,
                          parameter_msgpack_err_cb err_cb,
                          void *err_arg)
{
    uint32_t i;
    for (i = 0; i < map_size; i++) {
        uint32_t id_size;
        if (!cmp_read_str_size(cmp, &id_size)) {
            err_cb(err_arg, NULL, "could not read id");
            return -1;
        }
        char *id = PARAMETER_MSGPACK_MALLOC(id_size + 1); // +1 for nul term.
        if (id == NULL) {
            err_cb(err_arg, NULL, "allocation failed");
            return -1;
        }
        // read id string
        if (!cmp->read(cmp, id, id_size)) {
            err_cb(err_arg, NULL, "could not read id");
            PARAMETER_MSGPACK_FREE(id);
            return -1;
        }
        id[id_size] = '\0'; // nul termination for error printing
        cmp_object_t obj;
        if (!cmp_read_object(cmp, &obj)) {
            err_cb(err_arg, id, "could not read value");
            PARAMETER_MSGPACK_FREE(id);
            return -1;
        }
        if (cmp_object_is_map(&obj)) { // namespace
            parameter_namespace_t *sub = _parameter_namespace_find_w_id_len(ns, id , id_size);
            if (sub == NULL) {
                err_cb(err_arg, id, "warning: namespace doesn't exist");
            }
            PARAMETER_MSGPACK_FREE(id);
            if (sub != NULL) {
                uint32_t map_size;
                cmp_object_as_map(&obj, &map_size);
                int ret = read_namespace(sub, map_size, cmp, err_cb, err_arg);
                if (ret != 0) {
                    return ret;
                }
            } else {
                int ret = discard_msgpack_element(&obj, cmp, err_cb, err_arg, NULL);
                if (ret != 0) {
                    return ret;
                }
            }
        } else { // parameter
            parameter_t *p = _parameter_find_w_id_len(ns, id , id_size);
            if (p == NULL) {
                err_cb(err_arg, id, "warning: parameter doesn't exist");
            }
            PARAMETER_MSGPACK_FREE(id);
            if (p != NULL) {
                int ret = read_parameter(p, &obj, cmp, err_cb, err_arg);
                if (ret != 0) {
                    return ret;
                }
            } else { // parameter doesn't exist
                int ret = discard_msgpack_element(&obj, cmp, err_cb, err_arg, NULL);
                if (ret != 0) {
                    return ret;
                }
            }
        }
    }
    return 0;
}


static void err_ignore_cb(void *arg, const char *id, const char *err)
{
    (void)arg;
    (void)id;
    (void)err;
}


int parameter_msgpack_read_cmp(parameter_namespace_t *ns,
                               cmp_ctx_t *cmp,
                               parameter_msgpack_err_cb err_cb,
                               void *err_arg)
{
    if (err_cb == NULL) {
        err_cb = err_ignore_cb;
    }
    uint32_t map_size;
    if (!cmp_read_map(cmp, &map_size)) {
        err_cb(err_arg, NULL, "could not read namespace map");
        return -1;
    }
    return read_namespace(ns, map_size, cmp, err_cb, err_arg);
}


int parameter_msgpack_read(parameter_namespace_t *ns,
                           const void *buf,
                           size_t size,
                           parameter_msgpack_err_cb err_cb,
                           void *err_arg)
{
    cmp_ctx_t cmp;
    cmp_mem_access_t mem;
    cmp_mem_access_ro_init(&cmp, &mem, buf, size);
    return parameter_msgpack_read_cmp(ns, &cmp, err_cb, err_arg);
}


static void parameter_msgpack_write_subtree(const parameter_namespace_t *ns,
                                            cmp_ctx_t *cmp,
                                            parameter_msgpack_err_cb err_cb,
                                            void *err_arg)
{
    uint32_t map_size = 0;
    uint32_t i;

    parameter_namespace_t *child;
    parameter_t *param;
    bool success;

    for (child=ns->subspaces; child != NULL; child=child->next) {
        map_size ++;
    }

    for (param=ns->parameter_list; param!=NULL; param=param->next) {
        if (param->defined) {
            map_size ++;
        }
    }

    cmp_write_map(cmp, map_size);

    /* Write subtrees. */
    for (child=ns->subspaces; child != NULL; child=child->next) {
        cmp_write_str(cmp, child->id, strlen(child->id));
        parameter_msgpack_write_subtree(child, cmp, err_cb, err_arg);
    }

    /* Write each parameter. */
    for (param=ns->parameter_list; param!=NULL; param=param->next) {
        success = true;

        if (param->defined == false) {
            continue;
        }

        switch(param->type) {
            case _PARAM_TYPE_SCALAR:
                success &= cmp_write_str(cmp, param->id, strlen(param->id));
                success &= cmp_write_float(cmp, param->value.s);
                break;

            case _PARAM_TYPE_INTEGER:
                success &= cmp_write_str(cmp, param->id, strlen(param->id));
                success &= cmp_write_s32(cmp, param->value.i);
                break;

            case _PARAM_TYPE_BOOLEAN:
                success &= cmp_write_str(cmp, param->id, strlen(param->id));
                success &= cmp_write_bool(cmp, param->value.b);
                break;

            case _PARAM_TYPE_STRING:
                success &= cmp_write_str(cmp, param->id, strlen(param->id));
                success &= cmp_write_str(cmp, param->value.str.buf,
                                         param->value.str.len);
                break;

            case _PARAM_TYPE_VAR_VECTOR:
            case _PARAM_TYPE_VECTOR:
                success &= cmp_write_str(cmp, param->id, strlen(param->id));
                success &= cmp_write_array(cmp, param->value.vect.dim);
                for (i=0; i<param->value.vect.dim; i++) {
                    success &= cmp_write_float(cmp, param->value.vect.buf[i]);
                }
                break;

            default:
                err_cb(err_arg, param->id, "unsupported type for saving");
                break;
        }

        if (success == false) {
            err_cb(err_arg, param->id, "cmp_write failed");
        }
    }
}

void parameter_msgpack_write_cmp(const parameter_namespace_t *ns,
                                 cmp_ctx_t *cmp,
                                 parameter_msgpack_err_cb err_cb,
                                 void *err_arg)
{
    PARAMETER_LOCK();
    parameter_msgpack_write_subtree(ns, cmp, err_cb, err_arg);
    PARAMETER_UNLOCK();
}

/** Saves the given parameter tree to the given buffer as MessagePack. */
void parameter_msgpack_write(const parameter_namespace_t *ns,
                             void *buf,
                             size_t size,
                             parameter_msgpack_err_cb err_cb,
                             void *err_arg)
{
    cmp_ctx_t cmp;
    cmp_mem_access_t mem;
    cmp_mem_access_init(&cmp, &mem, buf, size);
    parameter_msgpack_write_cmp(ns, &cmp, err_cb, err_arg);
}
