#include "gui.h"
#include <ch.h>
#include <hal.h>
#include <string.h>
#include <stdio.h>
#include <gfx.h>
#include "main.h"

static MUTEX_DECL(gui_lock);
static GHandle score_label;
static GHandle console;
static bool init_done = false;
static void gui_thread(void *p)
{
    (void) p;

    chMtxLock(&gui_lock);
    gfxInit();
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
    chMtxUnlock(&gui_lock);

    WARNING("GUI init done");

    chThdSleepMilliseconds(1000);
    messagebus_topic_t *score_topic = messagebus_find_topic_blocking(&bus, "/score");
    while (true) {
        static char buffer[64];
        int score;
        messagebus_topic_wait(score_topic, &score, sizeof(score));
        sprintf(buffer, "Score: %d", score);
        chMtxLock(&gui_lock);
        gwinSetText(score_label, buffer, TRUE);
        chMtxUnlock(&gui_lock);
    }
}

void gui_start()
{
    static THD_WORKING_AREA(wa, 16384);
    chThdCreateStatic(wa, sizeof(wa), LOWPRIO, gui_thread, NULL);
}

void gui_log_console(struct error *e, va_list args)
{
    static char buffer[256];
    chMtxLock(&gui_lock);
    if (init_done) {
        char color;
        if (e->severity >= ERROR_SEVERITY_WARNING) {
            color = '3'; // yellow
        } else {
            color = 'C'; // no special color
        }
        snprintf(buffer, sizeof(buffer), "\n\033%c%c\033C  \033b%s:%d\033B  ",
                 color, *error_severity_get_name(e->severity),
                 strrchr(e->file, '/') + 1, e->line);
        gwinPrintf(console, buffer);
        vsnprintf(buffer, sizeof(buffer), e->text, args);
        gwinPrintf(console, buffer);
    }
    chMtxUnlock(&gui_lock);
}
