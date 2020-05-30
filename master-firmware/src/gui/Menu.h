#pragma once

#include "Page.h"
#include <gfx.h>

class Menu {
    Page* page;
    GHandle page_container;
    GHandle page_title;
    GHandle back_button;
    GTimer periodic_timer;
    GListener listener;

    /// Create page_container used to hold a page's content
    void create_container();

    void create_back_button();
    void create_page_title();

    /// Destroy page container and page elements
    void delete_container();

    /// Goes to the provided page, setting up title, back button and so on
    void switch_to_page(Page* page);

    /// Goes one page back in history
    void pop_page();

    /// Timer callback
    void on_timer();

public:
    Menu();

    /// Infinite loop processing events
    void event_loop();

    /// Enters provided page, handling history, window title, etc.
    void enter_page(Page* page);
};
