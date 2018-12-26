#ifndef GUI_MENU_H
#define GUI_MENU_H

#include "page.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    page_t* pages;
    int page_count;
} menu_t;

void menu_initialize(menu_t* menu);
void menu_load_page(menu_t* menu, int page_number);

#ifdef __cplusplus
}
#endif

#endif /* GUI_MENU_H */
