#pragma once

#include <gfx.h>

class Page {
    Page* history_previous;

public:
    /** Event called when we enter this page. */
    virtual void on_enter(GHandle parent) = 0;

    /** Event called when we exit this page. */
    virtual void on_exit()
    {
    }

    /** Event called periodically */
    virtual void on_timer()
    {
    }

    /** Called when an event occured. */
    virtual void on_event(GEvent* event) = 0;

    virtual const char* get_name() = 0;

    void set_previous_page(Page* p)
    {
        history_previous = p;
    }

    Page* get_previous_page() const
    {
        return history_previous;
    }

    virtual ~Page()
    {
    }
};
