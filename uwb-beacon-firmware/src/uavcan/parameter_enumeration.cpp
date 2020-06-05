#include <parameter/parameter.h>

static parameter_t* _parameter_find_by_index(parameter_namespace_t* root, int wanted, int* current)
{
    parameter_t* p = root->parameter_list;

    while (p) {
        if ((*current) == wanted) {
            return p;
        }
        (*current)++;
        p = p->next;
    }

    parameter_namespace_t* ns = root->subspaces;

    while (ns != NULL) {
        p = _parameter_find_by_index(ns, wanted, current);

        if (p) {
            return p;
        }

        ns = ns->next;
    }

    return NULL;
}

parameter_t* parameter_find_by_index(parameter_namespace_t* root, int index)
{
    int i = 0;
    return _parameter_find_by_index(root, index, &i);
}

int parameter_tree_height(parameter_t* leaf)
{
    int i = 0;
    parameter_namespace_s* ns = leaf->ns;

    while (ns != NULL) {
        i++;
        ns = ns->parent;
    }

    return i;
}
