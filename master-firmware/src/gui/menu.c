#include "menu.h"

void menu_initialize(menu_t* menu)
{
    for (int i = 0; i < menu->page_count; i++)
    {
        menu->pages[i].initialize(menu->pages[i].arg);
    }
}

void menu_load_page(menu_t* menu, int page_number)
{
    for (int i = 0; i < menu->page_count; i++)
    {
        menu->pages[i].delete(menu->pages[i].arg);
    }
    menu->pages[page_number].load(menu->pages[page_number].arg);
}
