#ifndef GUI_PAGE_POSITION_H
#define GUI_PAGE_POSITION_H

#include "gui/page.h"

#include <gfx.h>

typedef struct
{
    GHandle* label;
} page_position_t;

void page_position_init(void* arg);
void page_position_load(void *arg);
void page_position_delete(void *arg);

#endif /* GUI_PAGE_POSITION_H */