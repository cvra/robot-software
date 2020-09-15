#include <thread>
#include "gui.h"
#include <string.h>
#include <stdio.h>
#include <gfx.h>
#include "gui/Menu.h"
#include "gui/PositionPage.h"
#include "gui/MovePage.h"
#include "gui/ActuatorPage.h"
#include "gui/MenuPage.h"
#include "gui/ScorePage.h"
#include "main.h"

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

    auto front_left_page = ActuatorPage("actuator-front-left");
    auto front_center_page = ActuatorPage("actuator-front-center");
    auto front_right_page = ActuatorPage("actuator-front-right");
    auto back_left_page = ActuatorPage("actuator-back-left");
    auto back_center_page = ActuatorPage("actuator-back-left");
    auto back_right_page = ActuatorPage("actuator-back-right");

    auto actuator_menu = MenuPage(m, "Actuators",
                                  &front_left_page,
                                  &front_center_page,
                                  &front_right_page,
                                  &back_left_page,
                                  &back_center_page,
                                  &back_right_page);

    auto root_page = MenuPage(m, "Robot", &base_menu, &score_page, &actuator_menu);

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
