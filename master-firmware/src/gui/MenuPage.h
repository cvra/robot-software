#pragma once

#include <gfx.h>
#include "Menu.h"
#include "Page.h"

class MenuPage : public Page {
    GHandle buttons[6];
    Page* next_page[6];
    Menu& menu;
    const char* name;

public:
    /// Creates a page linking to up to 6 subpages
    MenuPage(Menu& menu,
             const char* name,
             Page* p1,
             Page* p2 = nullptr,
             Page* p3 = nullptr,
             Page* p4 = nullptr,
             Page* p5 = nullptr,
             Page* p6 = nullptr);

    virtual void on_enter(GHandle parent);

    virtual void on_event(GEvent* event);

    virtual const char* get_name()
    {
        return name;
    }
};
