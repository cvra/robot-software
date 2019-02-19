#include <math.h>

#include <math/geometry/discrete_circles.h>

bool discretize_circle(poly_t* poly, circle_t circle, int samples, float angle_offset)
{
    if (samples != poly->l)
        return false; // Number of samples doesn't match size of polygon

    float angle_increment = 2.f * M_PI / (float)samples;

    for (int i = 0; i < samples; i++) {
        float angle = i * angle_increment + angle_offset;
        poly->pts[i].x = circle.x + circle.r * cosf(angle);
        poly->pts[i].y = circle.y + circle.r * sinf(angle);
    }

    return true;
}
