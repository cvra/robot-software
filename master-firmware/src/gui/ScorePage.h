#pragma once

#include "gfx.h"

#include <stdlib.h>

#include "gui/Menu.h"
#include "gui/MenuPage.h"
#include "gui/Page.h"

#include <cstdio>

#ifndef GUI_SIMULATOR
#include <chibios-syscalls/stdio_lock.h>
#include "protobuf/strategy.pb.h"
#endif

class ScorePage : public Page {
    GHandle button;
    GHandle score_label;
    GHandle score_text;
    char score[10];

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
        score_label = gwinLabelCreate(0, &wi);
        gwinSetText(score_label, "Score:", gFalse);

        gwinWidgetClearInit(&wi);
        wi.g.width = SCREEN_WIDTH - 60;
        wi.g.height = 60;
        wi.g.y = 10;
        wi.g.x = 120;
        wi.g.parent = parent;
        wi.g.show = gTrue;
        score_text = gwinLabelCreate(0, &wi);
        gwinSetText(score_text, "0", gFalse);
    }

public:
    virtual const char* get_name() override
    {
        return "Score";
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
        stdio_lock();

#ifndef GUI_SIMULATOR
        auto topic = messagebus_find_topic(&bus, "/score");
        if (topic) {
            Score msg;
            if (messagebus_topic_read(topic, &msg, sizeof(msg))) {
                sprintf(score, "%ld", msg.score);
            }
        }
#endif

        stdio_unlock();
        gwinSetText(score_text, score, gFalse);
    }
};
