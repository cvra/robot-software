#include <math.h>

#include "lie_groups.h"

so2_t so2_create(float angle)
{
    so2_t rotation = {.angle = angle};
    return rotation;
}

point_t so2_rotate(so2_t rotation, point_t point)
{
    float cos_a = cosf(rotation.angle);
    float sin_a = sinf(rotation.angle);

    point_t res = {
        .x = cos_a * point.x - sin_a * point.y,
        .y = sin_a * point.x + cos_a * point.y,
    };

    return res;
}

point_t so2_inverse_rotate(so2_t rotation, point_t point)
{
    return so2_rotate(so2_create(-rotation.angle), point);
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

se2_t se2_create_xya(float x, float y, float angle)
{
    return se2_create(angle, translation_2d(x, y));
}

point_t se2_transform(se2_t transform, point_t point)
{
    point = so2_rotate(transform.rotation, point);

    point.x += transform.translation.x;
    point.y += transform.translation.y;

    return point;
}

point_t se2_inverse_transform(se2_t transform, point_t point)
{
    point.x -= transform.translation.x;
    point.y -= transform.translation.y;

    point = so2_inverse_rotate(transform.rotation, point);

    return point;
}

se2_t se2_chain(se2_t lhs, se2_t rhs)
{
    point_t t2 = {.x = rhs.translation.x, .y = rhs.translation.y};
    point_t translation = se2_transform(lhs, t2);

    se2_t res;
    res.rotation.angle = lhs.rotation.angle + rhs.rotation.angle;
    res.translation.x = translation.x;
    res.translation.y = translation.y;

    return res;
}

se2_t se2_inverse(se2_t transform)
{
    point_t translation = {.x = transform.translation.x, .y = transform.translation.y};
    translation = so2_inverse_rotate(transform.rotation, translation);

    se2_t res;
    res.rotation.angle = -transform.rotation.angle;
    res.translation.x = -translation.x;
    res.translation.y = -translation.y;

    return res;
}
