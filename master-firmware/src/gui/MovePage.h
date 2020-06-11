#pragma once
#include "gfx.h"
#include <error/error.h>

#include "gui/Menu.h"
#include "gui/MenuPage.h"
#include "gui/Page.h"
#include "absl/strings/str_cat.h"

#include "base/base_controller.h"
#include "base/base_helpers.h"

#define BUTTON_HEIGHT 70
#define BUTTON_WIDTH 225

class MovePage : public Page {
    GHandle forward_button, backward_button, plus_90_button, minus_90_button, center_table_button;

public:
    virtual const char* get_name() override
    {
        return "Move";
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
            wi.text = "Forward";
            forward_button = gwinButtonCreate(0, &wi);
        }
        {
            GWidgetInit wi;

            gwinWidgetClearInit(&wi);

            wi.g.show = gTrue;
            wi.g.parent = parent;
            wi.g.width = BUTTON_WIDTH;
            wi.g.height = BUTTON_HEIGHT;
            wi.g.y = 15 + 15 + BUTTON_HEIGHT;
            wi.g.x = 10;
            wi.text = "Backward";
            backward_button = gwinButtonCreate(0, &wi);
        }
        {
            GWidgetInit wi;

            gwinWidgetClearInit(&wi);

            wi.g.show = gTrue;
            wi.g.parent = parent;
            wi.g.width = BUTTON_WIDTH;
            wi.g.height = BUTTON_HEIGHT;
            wi.g.y = 15;
            wi.g.x = 10 + 10 + BUTTON_WIDTH;
            wi.text = "+90 deg";
            plus_90_button = gwinButtonCreate(0, &wi);
        }
        {
            GWidgetInit wi;

            gwinWidgetClearInit(&wi);

            wi.g.show = gTrue;
            wi.g.parent = parent;
            wi.g.width = BUTTON_WIDTH;
            wi.g.height = BUTTON_HEIGHT;
            wi.g.y = 15 + 15 + BUTTON_HEIGHT;
            wi.g.x = 10 + 10 + BUTTON_WIDTH;
            wi.text = "-90 deg";
            minus_90_button = gwinButtonCreate(0, &wi);
        }
        {
            GWidgetInit wi;

            gwinWidgetClearInit(&wi);

            wi.g.show = gTrue;
            wi.g.parent = parent;
            wi.g.width = BUTTON_WIDTH;
            wi.g.height = BUTTON_HEIGHT;
            wi.g.y = 15 + 2 * (15 + BUTTON_HEIGHT);
            wi.g.x = 10;
            wi.text = "Center";
            center_table_button = gwinButtonCreate(0, &wi);
        }
    }

    virtual void on_event(GEvent* event) override
    {
        if (event->type == GEVENT_GWIN_BUTTON) {
            auto wevent = reinterpret_cast<GEventGWinButton*>(event);
            if (wevent->gwin == forward_button) {
                NOTICE("clicked on move forward button");
                trajectory_d_rel(&robot.traj, 300);
            }
            if (wevent->gwin == backward_button) {
                NOTICE("clicked on move backward button");
                trajectory_d_rel(&robot.traj, -300);
            }
            if (wevent->gwin == plus_90_button) {
                NOTICE("clicked on +90 degrees button");
                trajectory_a_rel(&robot.traj, 90);
            }
            if (wevent->gwin == minus_90_button) {
                NOTICE("clicked on -90 degrees button");
                trajectory_a_rel(&robot.traj, -90);
            }
            if (wevent->gwin == center_table_button) {
                NOTICE("Going to the middle of the table");
                trajectory_goto_forward_xy_abs(&robot.traj, 1500, 1000);
            }
        }
    }

    virtual void on_timer() override
    {
    }
};
