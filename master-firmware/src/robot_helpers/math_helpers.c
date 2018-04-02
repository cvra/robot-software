#include <math.h>
#include "math_helpers.h"

#define MIN(a, b) (((a) < (b)) ? (a) : (b))
#define MAX(a, b) (((a) > (b)) ? (a) : (b))

float angle_delta(float start, float end)
{
    float res = fmodf(end - start + M_PI, 2 * M_PI) - M_PI;
    float res_conj = 2 * M_PI - res;

    if (fabsf(res) < fabsf(res_conj)) {
        return res;
    } else {
        return res_conj;
    }
}

bool math_point_is_in_square(poly_t* square, int x, int y)
{
    point_t point = {.x = x, .y = y};

    return is_in_poly(&point, square) == 1;
}

int math_clamp_value(int value, int min, int max)
{
    if (value < min) {
        return min;
    } else if (value > max) {
        return max;
    } else {
        return value;
    }
}

int argmin(float* values, int num_values)
{
    int min_index = 0;
    float min_value = INFINITY;
    for (int i = 0; i < num_values; i++) {
        if (values[i] < min_value) {
            min_index = i;
            min_value = values[i];
        }
    }
    return min_index;
}
