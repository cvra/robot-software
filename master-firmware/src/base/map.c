#include "math/geometry/discrete_circles.h"
#include "robot_helpers/math_helpers.h"
#include "base_controller.h"
#include "map.h"

#define TABLE_POINT_X(x) math_clamp_value(x, 0, MAP_SIZE_X_MM)
#define TABLE_POINT_Y(y) math_clamp_value(y, 0, MAP_SIZE_Y_MM)

static void map_lock(mutex_t *lock) { chMtxLock(lock); }
static void map_unlock(mutex_t *lock) { chMtxUnlock(lock); }

void map_init(struct _map* map, int robot_size)
{
    // Initialise obstacle avoidance state
    oa_init();
    chMtxObjectInit(&map->lock);

    /* Define table borders */
    polygon_set_boundingbox(robot_size/2, robot_size/2,
                            MAP_SIZE_X_MM - robot_size/2, MAP_SIZE_Y_MM - robot_size/2);

    /* Add opponent obstacle as points at origin */
    for (int i = 0; i < MAP_NUM_OPPONENT; i++) {
        map->opponents[i] = oa_new_poly(MAP_NUM_OPPONENT_EDGES);
        map_set_opponent_obstacle(map, i, 0, 0, 0, 0);
    }
    map->last_opponent_index = 0;

    /* Add wastewater obstacle */
    map->wastewater_obstacle = oa_new_poly(4);
    map_set_rectangular_obstacle(map->wastewater_obstacle, 1500, 1875, 1212, 250, robot_size);

    /* Setup cube obstacles */
    for (int i = 0; i < MAP_NUM_BLOCKS_CUBE; i++) {
        map->blocks_cube[i] = oa_new_poly(MAP_NUM_BLOCKS_CUBE_EDGES);
        map_set_cubes_obstacle(map, i, 0, 0, 0);
    }
}

void map_set_opponent_obstacle(struct _map* map, int index, int32_t x, int32_t y, int32_t opponent_size, int32_t robot_size)
{
    map_lock(&map->lock);
    map_set_rectangular_obstacle(map->opponents[index], x, y, opponent_size, opponent_size, robot_size);
    map_unlock(&map->lock);
}

poly_t* map_get_opponent_obstacle(struct _map* map, int index)
{
    return map->opponents[index];
}

void map_set_rectangular_obstacle(poly_t* opponent, int center_x, int center_y, int size_x, int size_y, int robot_size)
{
    opponent->pts[0].x = TABLE_POINT_X(center_x + (size_x + robot_size) / 2);
    opponent->pts[0].y = TABLE_POINT_Y(center_y - (size_y + robot_size) / 2);

    opponent->pts[1].x = TABLE_POINT_X(center_x + (size_x + robot_size) / 2);
    opponent->pts[1].y = TABLE_POINT_Y(center_y + (size_y + robot_size) / 2);

    opponent->pts[2].x = TABLE_POINT_X(center_x - (size_x + robot_size) / 2);
    opponent->pts[2].y = TABLE_POINT_Y(center_y + (size_y + robot_size) / 2);

    opponent->pts[3].x = TABLE_POINT_X(center_x - (size_x + robot_size) / 2);
    opponent->pts[3].y = TABLE_POINT_Y(center_y - (size_y + robot_size) / 2);
}

void map_update_opponent_obstacle(struct _map* map, int32_t x, int32_t y, int32_t opponent_size, int32_t robot_size)
{
    map_lock(&map->lock);
    map_set_rectangular_obstacle(map->opponents[map->last_opponent_index], x, y,
                                 opponent_size, opponent_size, robot_size);

    map->last_opponent_index++;
    if (map->last_opponent_index >= MAP_NUM_OPPONENT) {
        map->last_opponent_index = 0;
    }
    map_unlock(&map->lock);
}

poly_t* map_get_cubes_obstacle(struct _map* map, int index)
{
    return map->blocks_cube[index];
}

void map_set_cubes_obstacle(struct _map* map, int index, int x, int y, int robot_size)
{
    const int CUBES_BLOCK_SIZE = 180;
    circle_t circle = {.x = x, .y = y, .r = 0.5f * (CUBES_BLOCK_SIZE + robot_size)};

    map_lock(&map->lock);
    discretize_circle(map_get_cubes_obstacle(map, index), circle, MAP_NUM_BLOCKS_CUBE_EDGES, 0.125f * M_PI);
    map_unlock(&map->lock);
}
