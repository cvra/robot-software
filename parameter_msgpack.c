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

static int read_namespace(parameter_namespace_t *ns, cmp_ctx_t *cmp, cmp_mem_access_t *mem)
{
    uint32_t map_size;
    if (!cmp_read_map(cmp, &map_size)) {
        return -1;
    }
    uint32_t i;
    for (i = 0; i < map_size; i++) {
        // the map entry is  id: value
        // id_size is the lenght of the id string, pos_id the buffer position
        // of the id string, pos_value is the buffer position of the value
        uint32_t id_size;
        if (!cmp_read_str_size(cmp, &id_size)) {
            return -1; // read error or id was not a string
        }
        size_t pos_id = cmp_mem_access_get_pos(mem);
        size_t pos_value = pos_id + id_size;
        cmp_mem_access_set_pos(mem, pos_value); // jump id-string
        if (!cmp_mem_access_pos_is_valid(mem, pos_value - 1)) {
            return -1; // buffer ends in id string
        }
        char *id = cmp_mem_access_get_ptr_at_pos(mem, pos_id);
        cmp_object_t obj;
        if (!cmp_read_object(cmp, &obj)) {
            return -1;
        }
        if (cmp_object_is_map(&obj)) {
            parameter_namespace_t *sub = _parameter_namespace_find_w_id_len(ns, id , id_size);
            if (sub != NULL) {
                cmp_mem_access_set_pos(mem, pos_value); // reset to value
                int ret = read_namespace(sub, cmp, mem);
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

int parameter_msgpack_read(parameter_namespace_t *ns, const char *buf, size_t size)
{
    cmp_ctx_t cmp;
    cmp_mem_access_t mem;
    cmp_mem_access_ro_init(&cmp, &mem, buf, size);
    return read_namespace(ns, &cmp, &mem);
}
