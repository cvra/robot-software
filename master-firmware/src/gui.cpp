#include "gui.h"
#include <ch.h>
#include <hal.h>
#include <string.h>
#include <stdio.h>
#include <gfx.h>
#include "config.h"
#include "main.h"
#include "protobuf/strategy.pb.h"
#include "main.h"
#include "gui/Menu.h"
#include "gui/MenuPage.h"

class TestPage : public Page {
    GHandle button;
    const char* name;
    char msg[30];
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

        wi.g.show = true;
        wi.g.parent = parent;
        wi.g.width = 240;
        wi.g.height = 50;
        wi.g.y = 10;
        wi.g.x = 5;
        button = gwinButtonCreate(0, &wi);
        gwinSetText(button, "foobar", false);
    }

    virtual void on_timer()
    {
        sprintf(msg, "val: %d", counter);
        counter++;
        gwinSetText(button, msg, 0);
    }

    virtual void on_event(GEvent* event)
    {
    }
};

// from https://wiki.ugfx.io/index.php/Touchscreen_Calibration
gBool LoadMouseCalibration(unsigned instance, void* data, gMemSize sz)
{
    if (instance != 0) {
        return GFXOFF;
    }

    float calibrationData[6];
    calibrationData[0] = config_get_scalar("master/screen/ax");
    calibrationData[1] = config_get_scalar("master/screen/bx");
    calibrationData[2] = config_get_scalar("master/screen/cx");
    calibrationData[3] = config_get_scalar("master/screen/ay");
    calibrationData[4] = config_get_scalar("master/screen/by");
    calibrationData[5] = config_get_scalar("master/screen/cy");

    memcpy(data, (void*)&calibrationData, sz);

    return GFXON;
}

static void gui_thread(void* p)
{
    (void)p;

    gfxInit();
    gwinSetDefaultStyle(&WhiteWidgetStyle, GFXOFF);
    gwinSetDefaultFont(gdispOpenFont("DejaVuSans32"));
    gdispClear(Silver);
    gwinSetDefaultBgColor(Silver);

    WARNING("GUI init done");

    Menu m;
    auto base_menu = MenuPage(m, "Base", nullptr);
    auto arm_menu = MenuPage(m, "Arms", &base_menu);
    auto foo_menu = MenuPage(m, "foo", &base_menu);
    auto bar_menu = MenuPage(m, "bar", &base_menu);
    auto baz_menu = MenuPage(m, "baz", &base_menu);
    auto timer_test = TestPage("test");
    auto cvra_menu = MenuPage(m, "cvra", &base_menu, &timer_test);
    auto root_page = MenuPage(m, "Robot", &base_menu, &arm_menu, &foo_menu, &bar_menu, &baz_menu, &cvra_menu);

    m.enter_page(&root_page);
    m.event_loop();

    while (true) {
        chThdSleepMilliseconds(100);
    }
}

void gui_start()
{
    static THD_WORKING_AREA(wa, 4096);
    chThdCreateStatic(wa, sizeof(wa), LOWPRIO, gui_thread, NULL);
}
