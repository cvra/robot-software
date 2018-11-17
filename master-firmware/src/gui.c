#include "gui.h"
#include <ch.h>
#include <hal.h>
#include <string.h>
#include <stdio.h>
#include <gfx.h>
#include "config.h"
#include "main.h"
#include "protobuf/strategy.pb.h"

#include "gui_utilities.h"
#include "gui/menu.h"
#include "gui/pages/root.h"
#include "gui/pages/position.h"
#define COLOR_BACKGROUND Blue

static GHandle button_ts_menu;
static GHandle button_ts_page1;
static GHandle button_ts_page2;
static bool init_done = false;

#define MSG_MAX_LENGTH   128
#define MSG_BUF_SIZE     16

static char msg_buffer[MSG_MAX_LENGTH][MSG_BUF_SIZE];
static char *msg_mailbox_buf[MSG_BUF_SIZE];
static MAILBOX_DECL(msg_mailbox, msg_mailbox_buf, MSG_BUF_SIZE);
static MEMORYPOOL_DECL(msg_pool, MSG_MAX_LENGTH, NULL);

// from https://wiki.ugfx.io/index.php/Touchscreen_Calibration
bool_t LoadMouseCalibration(unsigned instance, void *data, size_t sz)
{
    if (instance != 0) {
        return FALSE;
    }

    float calibrationData[6];
    calibrationData[0] = config_get_scalar("master/screen/ax");
    calibrationData[1] = config_get_scalar("master/screen/bx");
    calibrationData[2] = config_get_scalar("master/screen/cx");
    calibrationData[3] = config_get_scalar("master/screen/ay");
    calibrationData[4] = config_get_scalar("master/screen/by");
    calibrationData[5] = config_get_scalar("master/screen/cy");

    memcpy(data, (void *)&calibrationData, sz);

    return TRUE;
}

static void gui_thread(void *p)
{
    (void)p;

    gfxInit();
    gwinSetDefaultStyle(&WhiteWidgetStyle, FALSE);
    gwinSetDefaultFont(gdispOpenFont("DejaVuSans12"));
    gdispClear(COLOR_BACKGROUND);
    {
        GWindowInit wi;
        memset(&wi, 0, sizeof(wi));
        wi.show = TRUE;
        wi.x = 0;
        wi.y = 0;
        wi.width = gdispGetWidth();
        wi.height = gdispGetHeight();
    }

    static GHandle label_ts_root;
    static page_root_t page_root_arg = {&label_ts_root};

    static GHandle label_ts_pos;
    static page_position_t page_position_arg = {&label_ts_pos};

    static page_t pages[] = {
        {&page_root_init, &page_root_load, &page_root_delete, &page_root_arg},
        {&page_position_init, &page_position_load, &page_position_delete, &page_position_arg},
    };
    menu_t my_menu = {pages, sizeof(pages) / sizeof(page_t)};
    menu_initialize(&my_menu);

    chPoolLoadArray(&msg_pool, msg_buffer, MSG_BUF_SIZE);
    init_done = true;

    WARNING("GUI init done");

    chThdSleepMilliseconds(1000);

    // We want to listen for widget events
    static GListener gl;
    geventListenerInit(&gl);
    gwinAttachListener(&gl);

    while (true) {
        // Get an Event
        GEvent *pe = geventEventWait(&gl, TIME_INFINITE);

        switch (pe->type) {
            case GEVENT_GWIN_BUTTON: {
                if (((GEventGWinButton *)pe)->gwin == button_ts_menu) {
                    menu_load_page(&my_menu, 0);
                } else if (((GEventGWinButton *)pe)->gwin == button_ts_page1) {
                    menu_load_page(&my_menu, 1);
                } else if (((GEventGWinButton *)pe)->gwin == button_ts_page2) {
                    menu_load_page(&my_menu, 2);
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
