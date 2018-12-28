#include "gfx.h"
#include "gui/Menu.h"
#include "gui/MenuPage.h"
#include "gui/Page.h"
#include <string.h>
#include <iostream>

class TestPage : public Page {
    GHandle button;
    const char* name;
    int counter;

public:
    TestPage(const char* name)
        : Page()
        , name(name)
        , counter(0)
    {
    }

    virtual const char* get_name()
    {
        return name;
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
        button = gwinLabelCreate(0, &wi);
        gwinSetText(button, "foobar", FALSE);
    }

    virtual void on_timer()
    {
        char msg[30];
        sprintf(msg, "x: %02d", counter);
        counter ++;
        gwinSetText(button, msg, TRUE);
    }

    virtual void on_event(GEvent* event)
    {
        std::cout << "click" << std::endl;
    }
};

int main(int argc, char* argv[])
{
    (void)argc;
    (void)argv;

    gfxInit();
    gdispClear(Silver);
    gwinSetDefaultBgColor(Silver);

    gwinSetDefaultFont(gdispOpenFont("DejaVuSans32"));

    Menu m;

    auto move_page = TestPage("move");
    auto odometry_page = TestPage("odometry");
    auto autoposition = TestPage("autoposition");
    auto base_menu = MenuPage(m, "Base", &move_page, &odometry_page, &autoposition);

    auto index_page = TestPage("index");
    auto pickup_puck = TestPage("pickup");
    auto arm_menu = MenuPage(m, "Arms", &index_page, &pickup_puck);

    auto align_page = TestPage("puck_align");
    auto pickup_action = TestPage("puck_pickup");
    auto actions = MenuPage(m, "Actions", &align_page, &pickup_action);
    auto strat_menu = MenuPage(m, "Strat", &actions);

    auto root_page = MenuPage(m, "Robot", &strat_menu, &base_menu, &arm_menu);
    m.enter_page(&root_page);
    m.event_loop();

    return 0;
}
