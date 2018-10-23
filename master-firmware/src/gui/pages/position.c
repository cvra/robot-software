#include "position.h"

#define COLOR_BACKGROUND Blue

static GHandle label_ts_test;
page_position_t page_position_arg = {&label_ts_test};

void page_position_init(void* arg)
{
    page_position_t *page = (page_position_t *)arg;

}

void page_position_load(void *arg)
{
    page_position_t *page = (page_position_t *)arg;
    gdispClear(COLOR_BACKGROUND);
}

void page_position_delete(void *arg)
{
    page_position_t *page = (page_position_t *)arg;
}
