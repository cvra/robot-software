#ifndef PARAMETER_ENUMERATION_HPP
#define PARAMETER_ENUMERATION_HPP

#include <parameter/parameter.h>

parameter_t* parameter_find_by_index(parameter_namespace_t* root, int index);
int parameter_tree_height(parameter_t* leaf);

#endif
