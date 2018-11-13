#ifndef GUI_MENU_H
#define GUI_MENU_H

#include "page.h"

typedef struct {
    page_t* pages;
    int page_count;
} menu_t;

void menu_initialize(menu_t* menu);
void menu_load_page(menu_t* menu, int page_number);

#endif /* GUI_MENU_H */
