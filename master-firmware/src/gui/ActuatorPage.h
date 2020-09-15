#pragma once
#include "gfx.h"
#include <error/error.h>

#include "gui/Menu.h"
#include "gui/MenuPage.h"
#include "gui/Page.h"
#include "absl/strings/str_cat.h"
#include "absl/strings/str_replace.h"
#include "protobuf/actuators.pb.h"

#include "main.h"

#include "can/actuator_driver.h"
class ActuatorPage : public Page {
    static const int BUTTON_HEIGHT = 70;
    static const int BUTTON_WIDTH = 145;

    GHandle pump0_button, pump1_button;
    GHandle digital_in_label;
    GHandle pressure0_label, pressure1_label;
    GHandle table_button, reef_button, high_button;
    std::string board_name;

public:
    ActuatorPage(const char* _name)
        : board_name(_name)
    {
    }

    virtual const char* get_name() override
    {
        std::string prefix = "actuator-";
        return board_name.c_str() + prefix.length();
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
        {
            GWidgetInit wi;

            gwinWidgetClearInit(&wi);

            wi.g.show = gTrue;
            wi.g.parent = parent;
            wi.g.width = BUTTON_WIDTH;
            wi.g.height = BUTTON_HEIGHT;
            wi.g.y = 15;
            wi.g.x = 10 + 2 * (10 + BUTTON_WIDTH);
            wi.text = "High";
            high_button = gwinButtonCreate(0, &wi);
        }
        {
            GWidgetInit wi;

            gwinWidgetClearInit(&wi);

            wi.g.show = gTrue;
            wi.g.parent = parent;
            wi.g.width = BUTTON_WIDTH;
            wi.g.height = BUTTON_HEIGHT;
            wi.g.y = 15 + (15 + BUTTON_HEIGHT);
            wi.g.x = 10 + 2 * (10 + BUTTON_WIDTH);
            wi.text = "Reef";
            reef_button = gwinButtonCreate(0, &wi);
        }
        {
            GWidgetInit wi;

            gwinWidgetClearInit(&wi);

            wi.g.show = gTrue;
            wi.g.parent = parent;
            wi.g.width = BUTTON_WIDTH;
            wi.g.height = BUTTON_HEIGHT;
            wi.g.y = 15 + 2 * (15 + BUTTON_HEIGHT);
            wi.g.x = 10 + 2 * (10 + BUTTON_WIDTH);
            wi.text = "Table";
            table_button = gwinButtonCreate(0, &wi);
        }
        {
            GWidgetInit wi;

            gwinWidgetClearInit(&wi);

            wi.g.show = gTrue;
            wi.g.parent = parent;
            wi.g.width = BUTTON_WIDTH;
            wi.g.height = BUTTON_HEIGHT;
            wi.g.y = 15 + 0 * (15 + BUTTON_HEIGHT);
            wi.g.x = 10 + 1 * (10 + BUTTON_WIDTH);
            wi.text = "XX kPa";
            pressure0_label = gwinLabelCreate(nullptr, &wi);
        }
        {
            GWidgetInit wi;

            gwinWidgetClearInit(&wi);

            wi.g.show = gTrue;
            wi.g.parent = parent;
            wi.g.width = BUTTON_WIDTH;
            wi.g.height = BUTTON_HEIGHT;
            wi.g.y = 15 + 1 * (15 + BUTTON_HEIGHT);
            wi.g.x = 10 + 1 * (10 + BUTTON_WIDTH);
            wi.text = "XX kPa";
            pressure1_label = gwinLabelCreate(nullptr, &wi);
        }

        {
            GWidgetInit wi;

            gwinWidgetClearInit(&wi);

            wi.g.show = gTrue;
            wi.g.parent = parent;
            wi.g.width = BUTTON_WIDTH;
            wi.g.height = BUTTON_HEIGHT;
            wi.g.y = 15 + 2 * (15 + BUTTON_HEIGHT);
            wi.g.x = 10 + 1 * (10 + BUTTON_WIDTH);
            wi.text = "UNKNOWN";
            digital_in_label = gwinLabelCreate(nullptr, &wi);
        }
    }

    virtual void on_event(GEvent* event) override
    {
        extern bus_enumerator_t bus_enumerator;
        auto* driver = reinterpret_cast<actuator_driver_t*>(bus_enumerator_get_driver(&bus_enumerator, board_name.c_str()));

        if (event->type == GEVENT_GWIN_BUTTON) {
            auto wevent = reinterpret_cast<GEventGWinButton*>(event);
            if (wevent->gwin == pump0_button) {
                actuator_driver_pump_set(driver, PUMP_TOP, !driver->pump_enabled[PUMP_TOP]);
            }
            if (wevent->gwin == pump1_button) {
                actuator_driver_pump_set(driver, PUMP_FRONT, !driver->pump_enabled[PUMP_FRONT]);
            }
            if (wevent->gwin == high_button) {
                actuator_driver_set_position(driver, POSITION_HIGH);
            }
            if (wevent->gwin == reef_button) {
                actuator_driver_set_position(driver, POSITION_REEF_PICKUP);
            }
            if (wevent->gwin == table_button) {
                actuator_driver_set_position(driver, POSITION_TABLE_PICKUP);
            }
        }
    }

    virtual void on_timer() override
    {
        std::string topic_name = absl::StrReplaceAll(board_name, {{"actuator-", "/actuator/"}});
        auto* topic = messagebus_find_topic(&bus, topic_name.c_str());

        ActuatorFeedback msg;

        if (!topic) {
            const char* err = "no topic";
            gwinSetText(pressure0_label, err, false);
            gwinSetText(pressure1_label, err, false);
            gwinSetText(digital_in_label, err, false);
            return;
        }

        bool success = messagebus_topic_read(topic, &msg, sizeof(msg));

        if (!success) {
            const char* err = "no data";
            gwinSetText(pressure0_label, err, false);
            gwinSetText(pressure1_label, err, false);
            gwinSetText(digital_in_label, err, false);
            return;
        }

        std::string text;

        text = absl::StrCat(msg.pressure[0] / 1000, " kPa");
        gwinSetText(pressure0_label, text.c_str(), true);

        text = absl::StrCat(msg.pressure[1] / 1000, " kPa");
        gwinSetText(pressure1_label, text.c_str(), true);

        if (msg.digital_input) {
            gwinSetText(digital_in_label, "HIGH", false);
        } else {
            gwinSetText(digital_in_label, "LOW", false);
        }
    }
};
