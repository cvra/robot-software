#ifndef GUI_PAGE_ROOT_H
#define GUI_PAGE_ROOT_H

#include "gui/page.h"

#include <gfx.h>

typedef struct
{
    GHandle* label;
} page_root_t;

extern page_root_t page_root_arg;

void page_root_init(void* arg);
void page_root_load(void *arg);
void page_root_delete(void *arg);

#endif /* GUI_PAGE_ROOT_H */
