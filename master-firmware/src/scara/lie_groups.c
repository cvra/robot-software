#include "lie_groups.h"

so2_t so2_create(float angle)
{
    so2_t rotation = {.angle = angle};
    return rotation;
}

point_t so2_rotate(so2_t rotation, point_t point)
{
    vect2_cart cartesian_point = {.x = point.x, .y = point.y};
    vect2_pol polar_point;

    vect2_cart2pol(&cartesian_point, &polar_point);
    polar_point.theta += rotation.angle;
    vect2_pol2cart(&polar_point, &cartesian_point);

    point.x = cartesian_point.x;
    point.y = cartesian_point.y;
    return point;
}

vect_t translation_2d(float x, float y)
{
    vect_t translation = {.x = x, .y = y};
    return translation;
}

se2_t se2_create(float angle, vect_t translation)
{
    se2_t transform = {.rotation = so2_create(angle), .translation = translation};
    return transform;
}

point_t se2_transform(se2_t transform, point_t point)
{
    point = so2_rotate(transform.rotation, point);

    point.x += transform.translation.x;
    point.y += transform.translation.y;

    return point;
}
