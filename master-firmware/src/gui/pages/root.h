#ifndef GUI_PAGE_ROOT_H
#define GUI_PAGE_ROOT_H

#include "gui/page.h"

#include <gfx.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    GHandle* label;
} page_root_t;

void page_root_init(void* arg);
void page_root_load(void* arg);
void page_root_delete(void* arg);

#ifdef __cplusplus
}
#endif

#endif /* GUI_PAGE_ROOT_H */
