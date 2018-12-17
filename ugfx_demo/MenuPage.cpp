#include "MenuPage.h"

MenuPage::MenuPage(Menu& menu,
                   const char* name,
                   Page* p1,
                   Page* p2,
                   Page* p3,
                   Page* p4,
                   Page* p5,
                   Page* p6)
    : Page()
    , name(name)
    , menu(menu)
{
    next_page[0] = p1;
    next_page[1] = p2;
    next_page[2] = p3;
    next_page[3] = p4;
    next_page[4] = p5;
    next_page[5] = p6;
}

void MenuPage::on_enter(GHandle parent)
{
    auto i = 0;
    for (auto x = 0; x < 2; x++) {
        for (auto y = 0; y < 3; y++) {
            if (!next_page[i]) {
                continue;
            }
            GWidgetInit wi;

            gwinWidgetClearInit(&wi);

            wi.g.show = TRUE;
            wi.g.parent = parent;
            wi.g.width = 225;
            wi.g.height = 70;
            wi.g.y = 15 + (wi.g.height + 15) * y;
            wi.g.x = 10 + (wi.g.width + 10) * x;
            buttons[i] = gwinButtonCreate(0, &wi);
            gwinSetText(buttons[i], next_page[i]->get_name(), FALSE);
            i++;
        }
    }
}

void MenuPage::on_event(GEvent* event)
{
    if (event->type == GEVENT_GWIN_BUTTON) {
        auto wevent = reinterpret_cast<GEventGWinButton*>(event);
        for (auto i = 0; i < 6; i++) {
            if (!next_page[i]) {
                continue;
            }
            if (wevent->gwin == buttons[i]) {
                menu.enter_page(next_page[i]);
            }
        }
    }
}
