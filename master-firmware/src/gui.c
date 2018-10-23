#include "gui.h"
#include <ch.h>
#include <hal.h>
#include <string.h>
#include <stdio.h>
#include <gfx.h>
#include "main.h"
#include "protobuf/strategy.pb.h"

#include "gui_utilities.h"
#define COLOR_BACKGROUND Blue

static GHandle label_ts_test;
static GHandle label_ts_test2;
static GHandle button_ts_menu;
static GHandle button_ts_page1;
static GHandle button_ts_page2;
static bool init_done = false;


// interface
struct page
{
    void (*load)(void*);
    void (*delete)(void*);
    void* arg;
};



//structure stockant les variables (data) à transmettre à la page 1
struct page_1
{
    GHandle* label;
    GHandle* label2;
};

struct page_1 page_1_arg = {&label_ts_test, &label_ts_test2};

// implémentation page 1
void page_1_load(void* arg)
{
    struct page_1* page = (struct page_1*)arg;
    gwinSetText(*page->label, "hope", TRUE);
    gdispClear(COLOR_BACKGROUND);
} 

void page_1_delete(void* arg){
    struct page_1* page = (struct page_1*)arg;
    gwinSetText(*page->label, "deleted", TRUE);
} 

// implémentation page 2
void page_2_load(void* arg){  

}
void page_2_delete(void* arg){

}
// implémentation page 3
void page_menu_load(void* arg){

} 

void page_menu_delete(void* arg){

}

// utilisation de l'interface
struct page pages[] = {
    {&page_1_load, &page_1_delete, &page_1_arg},
    {&page_2_load, &page_2_delete, NULL,},
    {&page_menu_load, &page_menu_delete, NULL}    
};

// -----------
struct menu {
    struct page *pages;
    int page_count;
};

struct menu my_menu = {pages, sizeof(pages) / sizeof(struct page)};

void menu_load_page (struct my menu, int page_number){

    for (int i=0; i<page_count; i++){
    my_menu.pages[i].delete(my_menu.pages[i].arg);
    }
    gdispClear(COLOR_BACKGROUND);
    my_menu.pages[page_number].load(my_menu.pages[page_number].arg);

}












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
    gdispClear(COLOR_BACKGROUND);
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
        gwinWidgetClearInit(&wi);
        wi.g.show = TRUE;
        wi.g.x = 50;
        wi.g.y = 45;
        wi.g.width = gdispGetWidth();
        wi.g.height = 40;
        label_ts_test = gwinLabelCreate(0, &wi);
        gwinSetFont(label_ts_test, gdispOpenFont("DejaVuSans32"));
        gwinSetText(label_ts_test, "Hand sensor", TRUE);
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
        wi.text = "Menu";
        button_ts_menu = gwinButtonCreate(0, &wi);
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
        wi.g.x = gdispGetWidth() - 105;
        wi.text = "page 1";
        button_ts_page1 = gwinButtonCreate(0, &wi);
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
        wi.text = "page 2";
        button_ts_page2 = gwinButtonCreate(0, &wi);
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
            if (((GEventGWinButton *)pe)->gwin == button_ts_menu)
            {
                delete_all_page(3);
                my_menu.pages[0].delete(my_menu.pages[0].arg);
            }
            else if (((GEventGWinButton *)pe)->gwin == button_ts_page1)
            {
                my_menu.pages[0].load(my_menu.pages[0].arg);
            }
            else if (((GEventGWinButton *)pe)->gwin == button_ts_page2)
            {
                gwinSetVisible(button_ts_page1, FALSE);
                gwinSetVisible(button_ts_menu, FALSE);
                gwinSetText(label_ts_test, "3", TRUE);
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
