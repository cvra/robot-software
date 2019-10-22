#pragma once
#include "gfx.h"

#include "gui/Menu.h"
#include "gui/MenuPage.h"
#include "gui/Page.h"

#include <cstdio>

#include "base/base_controller.h"
#include "base/base_helpers.h"
#include "chibios-syscalls/stdio_lock.h"
#include "protobuf/ally_position.pb.h"

static AllyPosition ally_get_position(void)
{
    AllyPosition pos = AllyPosition_init_zero;

    messagebus_topic_t* topic;
    const char* topic_name = "/ally_pos";

    topic = messagebus_find_topic(&bus, topic_name);
    if (topic != nullptr)
        messagebus_topic_read(topic, &pos, sizeof(pos));

    return pos;
}

class AllyPage : public Page {
    GHandle button;
    GHandle page_title;
    char msg[40];

    void create_label(GHandle parent)
    {
        GWidgetInit wi;
        gwinWidgetClearInit(&wi);
        wi.g.width = SCREEN_WIDTH - 60;
        wi.g.height = 60;
        wi.g.y = 10;
        wi.g.x = 10;
        wi.g.parent = parent;
        wi.g.show = gTrue;
        page_title = gwinLabelCreate(0, &wi);
    }

public:
    virtual const char* get_name() override
    {
        return "Ally";
    }

    virtual void on_enter(GHandle parent) override
    {
        create_label(parent);
    }

    virtual void on_event(GEvent*) override
    {
    }

    virtual void on_timer() override
    {
        AllyPosition pos = ally_get_position();
        stdio_lock();
        sprintf(msg, "x: %d y: %d a: %d deg", (int)pos.x, (int)pos.y, (int)pos.a);
        stdio_unlock();

        gwinSetText(page_title, msg, gFalse);
    }
};