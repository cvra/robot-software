#pragma once
#include "gfx.h"
#include <error/error.h>

#include "gui/Menu.h"
#include "gui/MenuPage.h"
#include "gui/Page.h"
#include "absl/strings/str_cat.h"

#include "can/actuator_driver.h"

#define BUTTON_HEIGHT 70
#define BUTTON_WIDTH 150

class ActuatorPage : public Page {
    GHandle pump0_button, pump1_button;
    actuator_driver_t* driver;
    char name[32];

public:
    ActuatorPage(actuator_driver_t* drv, const char* _name)
    {
        memset(name, 0, sizeof(name));
        strncpy(name, _name, sizeof(name) - 1);
        driver = drv;
    }

    virtual const char* get_name() override
    {
        return name;
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
            wi.text = "Pump 0";
            pump0_button = gwinButtonCreate(0, &wi);
        }
        {
            GWidgetInit wi;

            gwinWidgetClearInit(&wi);

            wi.g.show = gTrue;
            wi.g.parent = parent;
            wi.g.width = BUTTON_WIDTH;
            wi.g.height = BUTTON_HEIGHT;
            wi.g.y = 2 * 15 + BUTTON_HEIGHT;
            wi.g.x = 10;
            wi.text = "Pump 1";
            pump1_button = gwinButtonCreate(0, &wi);
        }
    }

    virtual void on_event(GEvent* event) override
    {
        if (event->type == GEVENT_GWIN_BUTTON) {
            auto wevent = reinterpret_cast<GEventGWinButton*>(event);
            if (wevent->gwin == pump0_button) {
                actuator_driver_pump_set(driver, PUMP_TOP, !driver->pump_enabled[PUMP_TOP]);
            }
            if (wevent->gwin == pump1_button) {
                actuator_driver_pump_set(driver, PUMP_FRONT, !driver->pump_enabled[PUMP_FRONT]);
            }
        }
    }

    virtual void on_timer() override
    {
    }
};
