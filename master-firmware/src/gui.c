#include "gui.h"
#include <ch.h>
#include <hal.h>
#include <string.h>
#include <stdio.h>
#include <gfx.h>

static GHandle score_label;
static GHandle console;
static bool init_done = false;
static void gui_thread(void *p)
{
    gfxInit();
    (void) p;
	gwinSetDefaultStyle(&WhiteWidgetStyle, FALSE);
    gwinSetDefaultFont(gdispOpenFont("DejaVuSans12"));
	gdispClear(White);
    {
        GWindowInit		wi;
        memset(&wi, 0, sizeof(wi));
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
    init_done= true;

    WARNING("GUI init done");
    while (true) {
        chThdSleepMilliseconds(100);
    }
}

void gui_start()
{
    static THD_WORKING_AREA(wa, 8192);
    chThdCreateStatic(wa, sizeof(wa), NORMALPRIO, gui_thread, NULL);
}

void gui_log_console(struct error *e, va_list args)
{
    static char buffer[256];
    if (init_done == false) {
        return;
    }
    if (e->severity >= ERROR_SEVERITY_WARNING) {
        snprintf(buffer, sizeof(buffer),
                 "\0333%c\033C ", *error_severity_get_name(e->severity));
    } else {
        snprintf(buffer, sizeof(buffer),
                     "%c ", *error_severity_get_name(e->severity));
    }
    gwinPrintf(console, buffer);
    snprintf(buffer, sizeof(buffer), "\033b%s:%d\033B  ", strrchr(e->file, '/') + 1, e->line);
    gwinPrintf(console, buffer);
    vsnprintf(buffer, sizeof(buffer), e->text, args);
    gwinPrintf(console, buffer);
    gwinPrintf(console, "\n");
}
