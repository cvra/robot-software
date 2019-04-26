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
#include "gui/ScorePage.h"
#include "gui/PositionPage.h"

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
    gdispClear(GFX_SILVER);
    gwinSetDefaultBgColor(GFX_SILVER);

    WARNING("GUI init done");

    Menu m;
    auto base_menu = MenuPage(m, "Base", nullptr);
    auto score_page = ScorePage();
    auto position_page = PositionPage();
    auto root_page = MenuPage(m, "Robot", &base_menu, &score_page, &position_page);

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
