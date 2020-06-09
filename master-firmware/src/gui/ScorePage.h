#pragma once
#include "gfx.h"
#include <error/error.h>

#include "gui/Menu.h"
#include "gui/MenuPage.h"
#include "gui/Page.h"
#include "absl/strings/str_cat.h"
#include "strategy/state.h"
#include "strategy/score.h"
#include "msgbus/messagebus.h"
#include "main.h"
#include "config.h"

class ScorePage : public Page {
    GHandle page_title;

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
        messagebus_topic_t* topic = messagebus_find_topic(&bus, "/state");

        if (topic == nullptr) {
            return;
        }

        StrategyState state;

        int score;
        if (messagebus_topic_read(topic, &state, sizeof state)) {
            bool is_main_robot = config_get_boolean("/master/is_main_robot");
            score = compute_score(state, is_main_robot);
        } else {
            score = -1;
        }

        std::string msg = absl::StrCat("Score: ", score);

        gwinSetText(page_title, msg.c_str(), true);
    }
};
