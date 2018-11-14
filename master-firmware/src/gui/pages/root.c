#include "root.h"

#define COLOR_BACKGROUND Blue

void page_root_init(void* arg)
{
    page_root_t *page = (page_root_t *)arg;
    GWidgetInit wi;
    gwinWidgetClearInit(&wi);
    wi.g.show = TRUE;
    wi.g.x = 50;
    wi.g.y = 45;
    wi.g.width = gdispGetWidth();
    wi.g.height = 40;
    *page->label = gwinLabelCreate(0, &wi);
    gwinSetFont(*page->label, gdispOpenFont("DejaVuSans32"));
    gwinSetText(*page->label, "Hand sensor", TRUE);
}

void page_root_load(void *arg)
{
    page_root_t *page = (page_root_t *)arg;
    gdispClear(COLOR_BACKGROUND);
    gwinSetText(*page->label, "hope", TRUE);
}

void page_root_delete(void *arg)
{
    page_root_t *page = (page_root_t *)arg;
    gwinSetText(*page->label, "deleted", TRUE);
}
