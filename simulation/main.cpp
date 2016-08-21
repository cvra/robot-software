#include <stdio.h>
#include <unistd.h>

extern "C" {
#include "math/geometry/polygon.h"
#include "obstacle_avoidance/obstacle_avoidance.h"
}

#include "visualizer.h"

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
        printf("Point #%d: %4.0lf %4.0lf\n", i, point->x, point->y);
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

    point_t bar[] = {{.x=1,.y=1},{.x=1.1,.y=1},{.x=1.1,.y=1.1},{.x=1,.y=1.1}};
    poly_t obstacles;
    obstacles.l = 4;
    obstacles.pts = bar;
    int nb_obstacles = 1;

    point_t traj[] = {{0.2,0.2},{1.2,0.9},{1.5,1.9}};
    int path_len = 3;

    visualizer_set_path(traj, path_len);
    visualizer_set_obstacles(&obstacles, nb_obstacles);

    visualizer_run();

    return 0;
}
