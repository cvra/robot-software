#include <ch.h>
#include <hal.h>

/* After how long without new orders on CAN we should start moving. */
#define SAFETY_TIMEOUT TIME_MS2I(500)

static virtual_timer_t safety_timer_timer;
static bool is_allowed_to_move = false;

static void _safety_timer_cb(void* p)
{
    (void)p;
    board_power_output_disable();
    is_allowed_to_move = false;

    /* unfortunately we cannot WARNING() from here as this is called from an
     * interrupt context. */
}

void safety_timer_restart(void)
{
    board_power_output_enable();
    is_allowed_to_move = true;
    chVTSet(&safety_timer_timer, SAFETY_TIMEOUT, _safety_timer_cb, NULL);
}

int safety_motion_is_allowed(void)
{
    return is_allowed_to_move;
}
