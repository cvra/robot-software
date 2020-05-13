#pragma once
#include "gfx.h"

#include "gui/Menu.h"
#include "gui/MenuPage.h"
#include "gui/Page.h"
#include "absl/strings/str_cat.h"

#include "base/base_controller.h"
#include "base/base_helpers.h"

class PositionPage : public Page {
    GHandle button;
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
        return "Position";
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
        auto x = 0, y = 0, a = 0;

        x = position_get_x_s16(&robot.pos);
        y = position_get_y_s16(&robot.pos);
        a = position_get_a_deg_s16(&robot.pos);

        std::string msg = absl::StrCat("x: ", x, " y: ", y, " a: ", a, " deg");

        gwinSetText(page_title, msg.c_str(), true);
    }
};
