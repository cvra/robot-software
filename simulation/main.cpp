#include <stdio.h>
#include <unistd.h>

extern "C" {
#include "math/geometry/polygon.h"
#include "obstacle_avoidance/obstacle_avoidance.h"
}

void print_obstacle_avoidance(struct obstacle_avoidance &oa)
{
    printf("--- Obstacle avoidance dump ---\n");

    printf("Number of Polygons: %d\n", oa.cur_poly_idx);
    for (int i = 0; i < oa.cur_poly_idx; i++) {
        poly_t *poly;
        poly = &oa.polys[i];
        printf("Polygon #%d\n", i);
        for (int j = 0; j < poly->l; j++) {
            point_t *pt;
            pt = &poly->pts[j];
            printf("  Point #%d (%4.0f,%4.0f)\n", j, pt->x, pt->y);
        }
    }

    printf("Number of Points: %d\n", oa.cur_pt_idx);
    for (int i = 0; i < oa.cur_pt_idx; i++) {
        point_t *point;
        point = &oa.points[i];
        printf("Point #%d: %d %d\n", i, point->x, point->y);
    }

    printf("\n");
}

int main(int argc, const char** argv)
{
    (void) argc;
    (void) argv;

    /* Set table */
    const int robot_size = 150;
    polygon_set_boundingbox(robot_size, robot_size, 3000-robot_size, 2000-robot_size);
    printf("Set the table bouding box: %d %d %d %d\n", robot_size, robot_size, 3000-robot_size, 2000-robot_size);

    oa_init();
    oa_reset();
    printf("Obstacle avoidance initialised");

    struct obstacle_avoidance oa;
    oa_copy(&oa);
    print_obstacle_avoidance(oa);

    return 0;
}
