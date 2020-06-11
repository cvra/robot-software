#include <thread>
#include "gui.h"
#include <string.h>
#include <stdio.h>
#include <gfx.h>
#include "gui/Menu.h"
#include "gui/PositionPage.h"
#include "gui/MovePage.h"
#include "gui/MenuPage.h"
#include "gui/ScorePage.h"

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
    PositionPage base_position_page;
    MovePage base_move_page;
    ScorePage score_page;
    auto base_menu = MenuPage(m, "Base", &base_position_page, &base_move_page);
    auto root_page = MenuPage(m, "Robot", &base_menu, &score_page);

    m.enter_page(&root_page);
    m.enter_page(&score_page);
    m.event_loop();

    while (true) {
    }
}

void gui_start()
{
    std::thread thd(gui_thread);
    thd.detach();
}
