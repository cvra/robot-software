#include "parameter_msgpack.h"
#include <cmp_mem_access/cmp_mem_access.h>

static int discard_msgpack_element(cmp_object_t *obj, cmp_ctx_t *cmp)
{
    return -5; // could not discard element
}


static int read_vector(cmp_ctx_t *cmp, float *buf, int len)
{
    return -1;
}

static int read_namespace(parameter_namespace_t *ns, uint32_t map_size, cmp_ctx_t *cmp)
{
    uint32_t i;
    for (i = 0; i < map_size; i++) {
        uint32_t id_size;
        if (!cmp_read_str_size(cmp, &id_size)) {
            return -1; // read error or id was not a string
        }
        char *id = malloc(id_size);
        if (id == NULL) {
            return -20;
        }
        // read id string
        if (!cmp->read(cmp, id, id_size)) {
            free(id);
            return -1;
        }
        cmp_object_t obj;
        if (!cmp_read_object(cmp, &obj)) {
            free(id);
            return -1;
        }
        if (cmp_object_is_map(&obj)) {
            parameter_namespace_t *sub = _parameter_namespace_find_w_id_len(ns, id , id_size);
            free(id);
            if (sub != NULL) {
                uint32_t map_size;
                cmp_object_as_map(&obj, &map_size);
                int ret = read_namespace(sub, map_size, cmp);
                if (ret != 0) {
                    return ret;
                }
            } else { // namespace doesn't exist
                int ret = discard_msgpack_element(&obj, cmp);
                if (ret != 0) {
                    return ret;
                }
            }
        } else {
            parameter_t *p = _parameter_find_w_id_len(ns, id , id_size);
            free(id);
            if (p != NULL) {
                if (p->type == _PARAM_TYPE_SCALAR) {
                    float valf;
                    int64_t vali;
                    if (cmp_object_as_float(&obj, &valf)) {
                        parameter_scalar_set(p, valf);
                    } else if (cmp_object_as_sinteger(&obj, &vali)) {
                        parameter_scalar_set(p, vali);
                    } else {
                        return -2; // type mismatch
                    }

                } else if (p->type == _PARAM_TYPE_VECTOR) {
                    uint32_t array_size;
                    if (cmp_object_as_array(&obj, &array_size)) {
                        if (array_size != p->value.vect.dim) {
                            return -2; // bad length
                        }
                        float *buf = malloc(array_size * sizeof(float));
                        if (buf == NULL) {
                            return -20; // allocation failed
                        }
                        int ret = read_vector(cmp, buf, array_size);
                        if (ret != 0) {
                            free(buf);
                            return ret;
                        }
                        parameter_vector_set(p, buf);
                        free(buf);
                    } else {
                        return -2; // type mismatch
                    }

                } else if (p->type == _PARAM_TYPE_VAR_VECTOR) {
                    uint32_t array_size;
                    if (cmp_object_as_array(&obj, &array_size)) {
                        if (array_size > p->value.vect.buf_dim) {
                            return -2; // too long
                        }
                        float *buf = malloc(array_size * sizeof(float));
                        if (buf == NULL) {
                            return -20; // allocation failed
                        }
                        int ret = read_vector(cmp, buf, array_size);
                        if (ret != 0) {
                            free(buf);
                            return ret;
                        }
                        parameter_variable_vector_set(p, buf, array_size);
                        free(buf);
                    } else {
                        return -2; // type mismatch
                    }

                } else {
                    return -3; // not implemented yet
                }
            } else { // parameter doesn't exist
                int ret = discard_msgpack_element(&obj, cmp);
                if (ret != 0) {
                    return ret;
                }
            }
        }
    }
    return 0;
}

int parameter_msgpack_read_cmp(parameter_namespace_t *ns, cmp_ctx_t *cmp)
{
    uint32_t map_size;
    if (!cmp_read_map(cmp, &map_size)) {
        return -1;
    }
    return read_namespace(ns, map_size, cmp);
}

int parameter_msgpack_read(parameter_namespace_t *ns, const char *buf, size_t size)
{
    cmp_ctx_t cmp;
    cmp_mem_access_t mem;
    cmp_mem_access_ro_init(&cmp, &mem, buf, size);
    return parameter_msgpack_read_cmp(ns, &cmp);
}
