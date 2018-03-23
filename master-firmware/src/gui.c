#include "gui.h"
#include <ch.h>
#include <hal.h>
#include <gfx.h>

static GHandle create_console(void)
{
    GWindowInit wi;
    const int console_height = 60;
    wi.show = TRUE;
    wi.x = 0;
    wi.y = console_height;
    wi.width = gdispGetWidth();
    wi.height = gdispGetHeight() - console_height;
    return gwinConsoleCreate(0, &wi);
}

static GHandle score_label;
static GHandle console;
static void gui_thread(void *p)
{
    gfxInit();
    (void) p;
	gwinSetDefaultStyle(&WhiteWidgetStyle, FALSE);
    gwinSetDefaultFont(gdispOpenFont("DejaVuSans12"));
	gdispClear(White);
    {
        GWindowInit		wi;
        wi.show = TRUE;
        wi.x = 0;
        wi.y = 40;
        wi.width = gdispGetWidth();
        wi.height = gdispGetHeight() - 40;
        console = gwinConsoleCreate(0, &wi);
    }
    {
        GWidgetInit		wi;
        memset(&wi, 0, sizeof(wi));
        wi.g.show = TRUE;
        wi.g.x = 0;
        wi.g.y = 0;
        wi.g.width = gdispGetWidth();
        wi.g.height = 40;
        score_label = gwinLabelCreate(0, &wi);
        gwinSetFont(score_label, gdispOpenFont("DejaVuSans32"));
        gwinSetText(score_label, "Score 42", TRUE);
    }
    gwinSetColor(console, White);
	gwinSetBgColor(console, Black);
	gwinClear(console);

    for (int i = 0; i < 10; i++) {
        gwinPrintf(console, "\033bstrategy.cpp:185: \033B INFO Trying to push the switch...\n");
        gwinPrintf(console, "\033bstrategy.cpp:204: \033B \0333WARNING\033C Failed, aborting!");
    }

    while (true) {
        chThdSleepMilliseconds(100);
    }
}

void gui_start()
{
    static THD_WORKING_AREA(wa, 8192);
    chThdCreateStatic(wa, sizeof(wa), NORMALPRIO, gui_thread, NULL);
}
