#include <stdio.h>
#include <unistd.h>

extern "C" {
#include "math/geometry/polygon.h"
#include "obstacle_avoidance/obstacle_avoidance.h"
}

#include "visualizer.h"

#define START_X  200
#define START_Y  1000
#define GOAL_X   2800
#define GOAL_Y   1000


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

void add_square_obstacle(int x, int y, int half_size)
{
    poly_t *obstacle = oa_new_poly(4);

    oa_poly_set_point(obstacle, x + half_size, y + half_size, 0);
    oa_poly_set_point(obstacle, x + half_size, y - half_size, 1);
    oa_poly_set_point(obstacle, x - half_size, y - half_size, 2);
    oa_poly_set_point(obstacle, x - half_size, y + half_size, 3);
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
    printf("Obstacle avoidance initialised\n");

    printf("Adding obstacles\n");
    add_square_obstacle(1000, 1200, 400);
    add_square_obstacle(2000,  800, 400);

    struct obstacle_avoidance oa;
    oa_copy(&oa);
    print_obstacle_avoidance(oa);

    printf("Visualizing obstacles\n");
    poly_t *obstacles = &oa.polys[1];
    int nb_obstacles = oa.cur_poly_idx - 1;

    oa_start_end_points(START_X, START_Y, GOAL_X, GOAL_Y);

    int path_len = oa_process();
    printf("Path of length %d\n", path_len);
    if (path_len < 0) {
        path_len = 0;
    }
    point_t *traj = oa_get_path();

    visualizer_set_path({START_X, START_Y}, traj, path_len);
    visualizer_set_obstacles(obstacles, nb_obstacles);

    visualizer_run();

    return 0;
}
