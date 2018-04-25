#include <string.h>

#include "wrist.h"

void wrist_init(wrist_t* wrist)
{
    memset(wrist, 0, sizeof(wrist_t));

    wrist->state = WRIST_HORIZONTAL;
}

void wrist_set_servo_callback(wrist_t *wrist, void (*set_servo)(void *, float), void *servo_args)
{
    wrist->set_servo = set_servo;
    wrist->servo_args = servo_args;
}

void wrist_set_servo_range(wrist_t *wrist, float horizontal, float vertical)
{
    wrist->servo_horizontal = horizontal;
    wrist->servo_vertical = vertical;
}

void wrist_set_horizontal(wrist_t* wrist)
{
    wrist->set_servo(wrist->servo_args, wrist->servo_horizontal);
    wrist->state = WRIST_HORIZONTAL;
}

void wrist_set_vertical(wrist_t* wrist)
{
    wrist->set_servo(wrist->servo_args, wrist->servo_vertical);
    wrist->state = WRIST_VERTICAL;
}
