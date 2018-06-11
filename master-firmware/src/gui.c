#include "gui.h"
#include <ch.h>
#include <hal.h>
#include <string.h>
#include <stdio.h>
#include <gfx.h>
#include "main.h"
#include "protobuf/strategy.pb.h"

static GHandle score_label;
static GHandle sensor_label;
static GHandle sensor_label2;
static GHandle console;
static GHandle   ghButton1;
static GHandle   ghCheckbox1;
static GHandle   ghSlider1;
static bool init_done = false;

#define MSG_MAX_LENGTH 128
#define MSG_BUF_SIZE   16
static char msg_buffer[MSG_MAX_LENGTH][MSG_BUF_SIZE];
static char *msg_mailbox_buf[MSG_BUF_SIZE];
static MAILBOX_DECL(msg_mailbox, msg_mailbox_buf, MSG_BUF_SIZE);
static MEMORYPOOL_DECL(msg_pool, MSG_MAX_LENGTH, NULL);

static void gui_thread(void *p)
{
    (void) p;

    gfxInit();
    gwinSetDefaultStyle(&WhiteWidgetStyle, FALSE);
    gwinSetDefaultFont(gdispOpenFont("DejaVuSans12"));
    gdispClear(White);
    {
        GWindowInit wi;
        memset(&wi, 0, sizeof(wi));
        wi.show = TRUE;
        wi.x = 0;
        wi.y = 40;
        wi.width = gdispGetWidth();
        wi.height = gdispGetHeight() - 40;
        console = gwinConsoleCreate(0, &wi);
    }
    {
        GWidgetInit wi;
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
        {
        GWidgetInit wi;
        gwinWidgetClearInit(&wi);
        memset(&wi, 0, sizeof(wi));
        wi.g.show = TRUE;
        wi.g.x = 0;
        wi.g.y = 50;
        wi.g.width = gdispGetWidth()/2  ;
        wi.g.height = 40;
        sensor_label = gwinLabelCreate(0, &wi);
        gwinSetFont(sensor_label, gdispOpenFont("DejaVuSans32"));
        gwinSetText(sensor_label, "Why dis not wark ?", TRUE);
    }
    {
        GWidgetInit wi;
        gwinWidgetClearInit(&wi);
        memset(&wi, 0, sizeof(wi));
        wi.g.show = TRUE;
        wi.g.x = 0;
        wi.g.y = 100;
        wi.g.width = gdispGetWidth()/2  ;
        wi.g.height = 40;
        sensor_label2 = gwinLabelCreate(0, &wi);
        gwinSetFont(sensor_label2, gdispOpenFont("DejaVuSans32"));
        gwinSetText(sensor_label2, "Score 47", TRUE);
    }
    /*

    {
	    GWidgetInit	wi;

        gwinWidgetClearInit(&wi);
        wi.g.show = TRUE;

        // Apply the button parameters
        wi.g.width = 100;
        wi.g.height = 30;
        wi.g.y = 100;
        wi.g.x = 10;
        wi.text = "Push Button";
	    ghButton1 = gwinButtonCreate(0, &wi);
    }

    {
        GWidgetInit	wi;

        // Apply some default values for GWIN
        gwinWidgetClearInit(&wi);
        wi.g.show = TRUE;

        // Apply the checkbox parameters
        wi.g.width = 100;		// includes text
        wi.g.height = 20;
        wi.g.y = 50;
        wi.g.x = 10;
        wi.text = "Checkbox";

        // Create the actual checkbox
        ghCheckbox1 = gwinCheckboxCreate(0, &wi);
    }

    {
	    GWidgetInit	wi;
	    gwinWidgetClearInit(&wi);
	    wi.g.show = TRUE;

        wi.g.width = 300;		// includes text
        wi.g.height = 50;
        wi.g.y = 150;
        wi.g.x = 10;
        wi.text = "Test";
	    ghSlider1 = gwinSliderCreate(0, &wi);
    }
    */

    gwinSetColor(console, White);
    gwinSetBgColor(console, Black);
    gwinClear(console);
    chPoolLoadArray(&msg_pool, msg_buffer, MSG_BUF_SIZE);
    init_done = true;

    WARNING("GUI init done");

    chThdSleepMilliseconds(1000);
    messagebus_topic_t *score_topic = messagebus_find_topic_blocking(&bus, "/score");
    while (true) {
        static char buffer[64];
        Score score_msg;
        if (messagebus_topic_read(score_topic, &score_msg, sizeof(score_msg))) {
            sprintf(buffer, "Score: %ld", score_msg.score);
            gwinSetText(score_label, buffer, TRUE);
        }

        char *msg;
        msg_t res = chMBFetch(&msg_mailbox, (msg_t *)&msg, MS2ST(500));
        if (res == MSG_OK) {
            gwinPrintf(console, msg);
            chPoolFree(&msg_pool, msg);
        }
    }
}

void gui_start()
{
    static THD_WORKING_AREA(wa, 4096);
    chThdCreateStatic(wa, sizeof(wa), LOWPRIO, gui_thread, NULL);
}

void gui_log_console(struct error *e, va_list args)
{
    static char buffer[256];
    if (init_done) {
        char *dst = chPoolAlloc(&msg_pool);
        if (dst) {
            dst[0] = '\0';
            char color;
            if (e->severity >= ERROR_SEVERITY_WARNING) {
                color = '3'; // yellow
            } else {
                color = 'C'; // no special color
            }
            snprintf(buffer, sizeof(buffer), "\n\033%c%c\033C  \033b%s:%d\033B  ",
                     color, *error_severity_get_name(e->severity),
                     strrchr(e->file, '/') + 1, e->line);
            strcat(dst, buffer);
            vsnprintf(buffer, sizeof(buffer), e->text, args);
            strcat(dst, buffer);
            chMBPost(&msg_mailbox, (msg_t)dst, TIME_IMMEDIATE);
        }
    }
}
