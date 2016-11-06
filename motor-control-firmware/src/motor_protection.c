#include <math.h>
#include "motor_protection.h"

#define T_AMBIENT 25

void motor_protection_init(motor_protection_t *p, float t_max, float r_th, float c_th, float current_gain)
{
    p->t = T_AMBIENT;
    p->t_ambient = T_AMBIENT;
    p->t_max = t_max;
    p->r_th = r_th;
    p->c_th = c_th;
    p->current_gain = current_gain;
}

float motor_protection_update(motor_protection_t *p, float current, float delta_t)
{
    float P_in = current * p->current_gain;
    float P_out = (p->t - p->t_ambient) / p->r_th;
    p->t += (P_in - P_out) * delta_t / p->c_th;

    float equilibrium_current = P_out / p->current_gain; // P_in = P_out
    if (p->t >= p->t_max) {
        return equilibrium_current;
    }
    return INFINITY;
}
