#include "gui.h"
#include <ch.h>
#include <hal.h>
#include <string.h>
#include <stdio.h>
#include <gfx.h>
#include "main.h"
#include "protobuf/strategy.pb.h"

#include "gui_utilities.h"

static GHandle score_label;
static GHandle sensor_label;
static GHandle sensor2_label;
static GHandle console;
static GHandle ghButton1;
static GHandle ghButton2;
static GHandle ghButton3;
static GHandle ghCheckbox1;
static GHandle ghSlider1;
static bool init_done = false;

#define MSG_MAX_LENGTH 128
#define MSG_BUF_SIZE 16
static char msg_buffer[MSG_MAX_LENGTH][MSG_BUF_SIZE];
static char *msg_mailbox_buf[MSG_BUF_SIZE];
static MAILBOX_DECL(msg_mailbox, msg_mailbox_buf, MSG_BUF_SIZE);
static MEMORYPOOL_DECL(msg_pool, MSG_MAX_LENGTH, NULL);

float calibrationData[] = {
    0.089231252,   // ax
    0.000522375,   // bx
    -22.532257080, // cx
    -0.001139728,  // ay
    0.127720847,   // by
    -28.272106170, // cy
};

// from https://wiki.ugfx.io/index.php/Touchscreen_Calibration
bool_t LoadMouseCalibration(unsigned instance, void *data, size_t sz)
{
    (void)instance;

    if (sz != sizeof(calibrationData) || instance != 0)
    {
        return FALSE;
    }

    memcpy(data, (void *)&calibrationData, sz);

    return TRUE;
}

static void gui_thread(void *p)
{
    (void)p;

    gfxInit();
    gwinSetDefaultStyle(&WhiteWidgetStyle, FALSE);
    gwinSetDefaultFont(gdispOpenFont("DejaVuSans12"));
    gdispClear(White);
    {
        GWindowInit wi;
        memset(&wi, 0, sizeof(wi));
        wi.show = TRUE;
        wi.x = 0;
        wi.y = 0;
        wi.width = gdispGetWidth();
        wi.height = gdispGetHeight();
        //console = gwinConsoleCreate(0, &wi);
    }

    {
        GWidgetInit wi;
                memset(&wi, 0, sizeof(wi));


        gwinWidgetClearInit(&wi);
        wi.g.show = TRUE;

        // Apply the button parameters
        wi.g.width = 100;
        wi.g.height = 25;
        wi.g.y = 0;
        wi.g.x = 5;
        wi.text = "1";
        ghButton1 = gwinButtonCreate(0, &wi);
    }
    {
        GWidgetInit wi;
                memset(&wi, 0, sizeof(wi));


        gwinWidgetClearInit(&wi);
        wi.g.show = TRUE;

        // Apply the button parameters
        wi.g.width = 100;
        wi.g.height = 25;
        wi.g.y = 0;
        wi.g.x = 20;
        wi.text = "2";
        ghButton2 = gwinButtonCreate(0, &wi);
    }
    {
        GWidgetInit wi;
                memset(&wi, 0, sizeof(wi));


        gwinWidgetClearInit(&wi);
        wi.g.show = TRUE;

        // Apply the button parameters
        wi.g.width = 100;
        wi.g.height = 25;
        wi.g.y = 0;
        wi.g.x = gdispGetWidth() / 2 - 50;
        wi.text = "3";
        ghButton3 = gwinButtonCreate(0, &wi);
    }

    
    chPoolLoadArray(&msg_pool, msg_buffer, MSG_BUF_SIZE);
    init_done = true;

    WARNING("GUI init done");

    NOTICE("test %d", 42);

    chThdSleepMilliseconds(1000);

    // We want to listen for widget events
    static GListener gl;
    geventListenerInit(&gl);
    gwinAttachListener(&gl);

    while (true)
    {
        // Get an Event
        GEvent *pe = geventEventWait(&gl, TIME_INFINITE);

        switch (pe->type)
        {
        case GEVENT_GWIN_BUTTON:
        {
            if (((GEventGWinButton *)pe)->gwin == ghButton1)
            {
                // Our button has been pressed
                NOTICE("been pressed 1");
                gwinSetVisible(ghButton2, TRUE);
                gwinSetVisible(ghButton1, FALSE);

            }
            else if (((GEventGWinButton *)pe)->gwin == ghButton2)
            {
                // Our button has been pressed
                NOTICE("been pressed 2");
                gwinSetVisible(ghButton1, TRUE);
                gwinSetVisible(ghButton2, FALSE);
            }
            else if (((GEventGWinButton *)pe)->gwin == ghButton3)
            {
                // Our button has been pressed
                NOTICE("been pressed 3");
                gwinDestroy(ghButton3);
            }
        }
        break;
        default:
            break;
        }
        chThdSleepMilliseconds(100);
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
    return;
    if (init_done)
    {
        char *dst = chPoolAlloc(&msg_pool);
        if (dst)
        {
            dst[0] = '\0';
            char color;
            if (e->severity >= ERROR_SEVERITY_WARNING)
            {
                color = '3'; // yellow
            }
            else
            {
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
