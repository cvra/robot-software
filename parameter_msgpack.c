#include "parameter_msgpack.h"
#include <cmp_mem_access/cmp_mem_access.h>

static int read_with_cmp_mem_access(parameter_namespace_t *ns, cmp_ctx_t *cmp, cmp_mem_access_t *mem)
{
    uint32_t map_size;
    if (!cmp_read_map(cmp, &map_size)) {
        return -1;
    }
    uint32_t i;
    for (i = 0; i < map_size; i++) {
        uint32_t id_size;
        if (!cmp_read_str_size(cmp, &id_size)) {
            return -1;
        }
        size_t id_pos = cmp_mem_access_get_pos(mem);
        cmp_mem_access_set_pos(mem, id_pos + id_size); // jump id-string
        if (!cmp_mem_access_pos_is_valid(mem, id_pos + id_size - 1)) {
            return -1;
        }
        char *id = cmp_mem_access_get_ptr_at_pos(mem, id_pos);
        cmp_object_t obj;
        if (!cmp_read_object(cmp, &obj)) {
            return -1;
        }
        if (cmp_object_is_map(&obj)) {
            parameter_namespace_t *sub = _parameter_namespace_find_w_id_len(ns, id , id_size);
            if (sub != NULL) {
                cmp_mem_access_set_pos(mem, id_pos + id_size); // reset again
                int ret = read_with_cmp_mem_access(sub, cmp, mem);
                if (ret != 0) {
                    return ret;
                }
            } else {
                return -4; // namespace not found
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
                } else {
                    return -3; // not implemented yet
                }
            } else {
                return -5; // parameter not found
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
    return read_with_cmp_mem_access(ns, &cmp, &mem);
}
