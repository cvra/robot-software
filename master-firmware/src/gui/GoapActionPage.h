#pragma once

#include "gfx.h"
#include <error/error.h>

#include "gui/Menu.h"
#include "gui/MenuPage.h"
#include "gui/Page.h"
#include "absl/strings/str_cat.h"
#include "goap/goap.hpp"
#include "strategy/actions.h"

class GoapActionPage : public Page {
    actions::NamedAction<StrategyState>& action_;
    std::string name_;
    static const int BUTTON_HEIGHT = 70;
    static const int BUTTON_WIDTH = 225;
    GHandle run_button;

public:
    GoapActionPage(actions::NamedAction<StrategyState>& action)
        : Page()
        , action_(action)
        , name_(action.get_name())
    {
    }

    const char* get_name() override
    {
        return name_.c_str();
    }

    virtual void on_enter(GHandle parent) override
    {
        {
            GWidgetInit wi;

            gwinWidgetClearInit(&wi);

            wi.g.show = gTrue;
            wi.g.parent = parent;
            wi.g.width = BUTTON_WIDTH;
            wi.g.height = BUTTON_HEIGHT;
            wi.g.y = 15;
            wi.g.x = 10;
            wi.text = "Run";
            run_button = gwinButtonCreate(0, &wi);
        }
    }

    virtual void on_event(GEvent* event) override
    {
        if (event->type == GEVENT_GWIN_BUTTON) {
            auto wevent = reinterpret_cast<GEventGWinButton*>(event);
            if (wevent->gwin == run_button) {
                // TODO(antoinealb): Read state from messagebus
                StrategyState state;
                action_.execute(state);
            }
        }
    }
};
