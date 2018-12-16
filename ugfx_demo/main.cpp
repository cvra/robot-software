#include "gfx.h"
#include <string.h>
#include <iostream>

class Menu;

class Page {
    Page* history_previous;

public:
    /** Event called when we enter this page. */
    virtual void on_enter(GHandle parent) = 0;

    /** Event called when we exit this page. */
    virtual void on_exit()
    {
    }

    /** Called when an event occured. */
    virtual void on_event(GEvent* event) = 0;

    virtual const char* get_name() = 0;

    void set_previous_page(Page* p)
    {
        history_previous = p;
    }

    Page* get_previous_page() const
    {
        return history_previous;
    }
};

class TestPage : public Page {
    GHandle button;

public:
    virtual const char* get_name()
    {
        return "test_page";
    }

    virtual void on_enter(GHandle parent)
    {
        GWidgetInit wi;

        gwinWidgetClearInit(&wi);

        wi.g.show = TRUE;
        wi.g.parent = parent;
        wi.g.width = 100;
        wi.g.height = 50;
        wi.g.y = 10;
        wi.g.x = 5;
        button = gwinButtonCreate(0, &wi);
        gwinSetText(button, "foobar", FALSE);
    }

    virtual void on_event(GEvent* event)
    {
        std::cout << "click" << std::endl;
    }
};

class Menu {
    Page* page;
    GHandle page_container;
    GHandle page_title;
    GHandle back_button;
    GListener listener;

    void create_container()
    {
        GWidgetInit wi;
        gwinWidgetClearInit(&wi);
        wi.g.width = GDISP_SCREEN_WIDTH;
        wi.g.height = GDISP_SCREEN_HEIGHT - 60;
        wi.g.y = 60;
        wi.g.x = 0;
        wi.g.show = TRUE;
        wi.customDraw = gwinContainerDraw_Transparent;
        page_container = gwinContainerCreate(0, &wi, 0);
    }

    void delete_container()
    {
        gwinDestroy(page_container);
    }

    void pop_page()
    {
        delete_container();
        create_container();
        page = page->get_previous_page();
        this->page->on_enter(page_container);
        gwinSetText(page_title, page->get_name(), FALSE);
    }

public:
    Menu()
        : page(nullptr)
    {
        create_container();
        {
            GWidgetInit wi;
            gwinWidgetClearInit(&wi);
            wi.g.width = GDISP_SCREEN_WIDTH - 60;
            wi.g.height = 60;
            wi.g.y = 0;
            wi.g.x = 60;
            wi.g.show = TRUE;
            page_title = gwinLabelCreate(0, &wi);
        }

        {
            GWidgetInit wi;
            gwinWidgetClearInit(&wi);
            wi.g.width = 40;
            wi.g.height = 40;
            wi.g.y = 10;
            wi.g.x = 10;
            wi.g.show = TRUE;
            wi.customDraw = gwinButtonDraw_ArrowLeft;
            back_button = gwinButtonCreate(0, &wi);
        }
        geventListenerInit(&listener);
        gwinAttachListener(&listener);
    }

    void on_enter()
    {
        page->on_enter(page_container);
        gwinSetText(page_title, page->get_name(), FALSE);

        while (true) {
            GEvent* event = geventEventWait(&listener, TIME_INFINITE);

            if (event->type == GEVENT_GWIN_BUTTON && reinterpret_cast<GEventGWinButton*>(event)->gwin == back_button) {
                pop_page();
            } else {
                page->on_event(event);
            }
        }
    }

    void enter_page(Page* page)
    {
        delete_container();
        create_container();
        page->set_previous_page(this->page);
        this->page = page;

        gwinSetText(page_title, page->get_name(), FALSE);
        this->page->on_enter(page_container);
    }
};

class MenuPage : public Page {
    GHandle buttons[6];
    Page* next_page[6];
    Menu& menu;
    const char* name;

public:
    MenuPage(Menu& menu,
             const char* name,
             Page* p1,
             Page* p2 = nullptr,
             Page* p3 = nullptr,
             Page* p4 = nullptr,
             Page* p5 = nullptr,
             Page* p6 = nullptr)
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

    virtual void on_enter(GHandle parent)
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

    virtual void on_event(GEvent* event)
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

    virtual const char* get_name()
    {
        return name;
    }
};

int main(int argc, char* argv[])
{
    (void)argc;
    (void)argv;

    gfxInit();
    gdispClear(Silver);
    gwinSetDefaultBgColor(White);

    gwinSetDefaultFont(gdispOpenFont("DejaVuSans32"));

    auto p = TestPage();
    Menu m;
    auto mp = MenuPage(m, "Robot", &p, &p, &p, &p);
    m.enter_page(&mp);
    m.on_enter();

    while (1) {
        gfxSleepMilliseconds(1000);
    }

    return 0;
}
