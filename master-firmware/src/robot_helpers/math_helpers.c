#include <math.h>
#include "math_helpers.h"

#define MIN(a,b) (((a)<(b))?(a):(b))
#define MAX(a,b) (((a)>(b))?(a):(b))

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
    int x_min = MIN(MIN(square->pts[0].x, square->pts[1].x), MIN(square->pts[2].x, square->pts[3].x));
    int x_max = MAX(MAX(square->pts[0].x, square->pts[1].x), MAX(square->pts[2].x, square->pts[3].x));
    int y_min = MIN(MIN(square->pts[0].y, square->pts[1].y), MIN(square->pts[2].y, square->pts[3].y));
    int y_max = MAX(MAX(square->pts[0].y, square->pts[1].y), MAX(square->pts[2].y, square->pts[3].y));

    return (x > x_min && x < x_max && y > y_min && y < y_max);
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
