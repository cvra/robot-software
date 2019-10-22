#include "Menu.h"
#include <gfx.h>

Menu::Menu()
    : page(nullptr)
{
    create_container();
    create_back_button();
    create_page_title();

    geventListenerInit(&listener);
    gwinAttachListener(&listener);

#ifndef GUI_SIMULATOR
    chMtxObjectInit(&menu_lock);
#endif
}

void Menu::create_container()
{
    GWidgetInit wi;
    gwinWidgetClearInit(&wi);
    wi.g.width = SCREEN_WIDTH;
    wi.g.height = SCREEN_HEIGHT - 60;
    wi.g.y = 60;
    wi.g.x = 0;
    wi.g.show = gTrue;
    wi.customDraw = gwinContainerDraw_Transparent;
    page_container = gwinContainerCreate(0, &wi, 0);
}

void Menu::delete_container()
{
    gwinDestroy(page_container);
}

void Menu::switch_to_page(Page* page)
{
    delete_container();
    create_container();
    this->page = page;

    if (this->page->get_previous_page()) {
        gwinEnable(back_button);
    } else {
        gwinDisable(back_button);
    }
    this->page->on_enter(page_container);
    gwinSetText(page_title, page->get_name(), gFalse);
}

void Menu::pop_page()
{
    switch_to_page(page->get_previous_page());
}

void Menu::on_timer()
{
    page->on_timer();
}

void Menu::enter_page(Page* page)
{
    page->set_previous_page(this->page);
    switch_to_page(page);
}

void Menu::event_loop()
{
    gtimerInit(&periodic_timer);
    gtimerStart(
        &periodic_timer, [](void* p) {
            Menu* m = reinterpret_cast<Menu*>(p);
#ifndef GUI_SIMULATOR
            chMtxLock(&m->menu_lock);
#endif
            m->on_timer();
#ifndef GUI_SIMULATOR
            chMtxUnlock(&m->menu_lock);
#endif
        },
        this, gTrue, 100);

    while (true) {
        GEvent* event = geventEventWait(&listener, gDelayForever);

#ifndef GUI_SIMULATOR
        chMtxLock(&menu_lock);
#endif
        if (event->type == GEVENT_GWIN_BUTTON && reinterpret_cast<GEventGWinButton*>(event)->gwin == back_button) {
            pop_page();
        } else {
            page->on_event(event);
        }
#ifndef GUI_SIMULATOR
        chMtxUnlock(&menu_lock);
#endif
    }
}

void Menu::create_back_button()
{
    GWidgetInit wi;
    gwinWidgetClearInit(&wi);
    wi.g.width = 40;
    wi.g.height = 40;
    wi.g.y = 10;
    wi.g.x = 10;
    wi.g.show = gTrue;
    wi.customDraw = gwinButtonDraw_ArrowLeft;
    back_button = gwinButtonCreate(0, &wi);
}

void Menu::create_page_title()
{
    GWidgetInit wi;
    gwinWidgetClearInit(&wi);
    wi.g.width = SCREEN_WIDTH - 60;
    wi.g.height = 60;
    wi.g.y = 0;
    wi.g.x = 60;
    wi.g.show = gTrue;
    page_title = gwinLabelCreate(0, &wi);
}
