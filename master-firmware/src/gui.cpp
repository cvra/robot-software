#include <thread>
#include "gui.h"
#include <string.h>
#include <stdio.h>
#include <gfx.h>
#include "gui/Menu.h"
#include "gui/PositionPage.h"
#include "gui/MenuPage.h"

// from https://wiki.ugfx.io/index.php/Touchscreen_Calibration
gBool LoadMouseCalibration(unsigned instance, void* data, gMemSize sz)
{
    return GFXON;
}

static void gui_thread()
{
    gfxInit();
    gwinSetDefaultStyle(&WhiteWidgetStyle, GFXOFF);
    gwinSetDefaultFont(gdispOpenFont("DejaVuSans32"));
    gdispClear(GFX_SILVER);
    gwinSetDefaultBgColor(GFX_SILVER);
    gdispSetOrientation(gOrientation90);

    WARNING("GUI init done");

    Menu m;
    auto position_page = PositionPage();
    auto base_menu = MenuPage(m, "Base", &position_page);
    auto root_page = MenuPage(m, "Robot", &base_menu);

    m.enter_page(&root_page);
    m.event_loop();

    while (true) {
    }
}

void gui_start()
{
    std::thread thd(gui_thread);
    thd.detach();
}
