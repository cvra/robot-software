#include "robot_helpers/math_helpers.h"
#include "base_controller.h"
#include "map.h"

static struct _map map;

#define TABLE_POINT_X(x) math_clamp_value(x, 0, MAP_SIZE_X_MM)
#define TABLE_POINT_Y(y) math_clamp_value(y, 0, MAP_SIZE_Y_MM)

void map_init(int robot_size)
{
    // Initialise obstacle avoidance state
    oa_init();

    /* Define table borders */
    polygon_set_boundingbox(robot_size/2, robot_size/2,
                            MAP_SIZE_X_MM - robot_size/2, MAP_SIZE_Y_MM - robot_size/2);

    /* Add contruction areas */
    for (int i = 0; i < MAP_NUM_CONSTRUCTION_AREA; i++) {
        map.construction_area[i] = oa_new_poly(4);
    }
    map_set_rectangular_obstacle(map.construction_area[0], 925, 110, 110, 500, robot_size);
    map_set_rectangular_obstacle(map.construction_area[1], 925, 2890, 110, 500, robot_size);

    /* Add craters */
    for (int i = 0; i < MAP_NUM_CRATER; i++) {
        map.crater[i] = oa_new_poly(4);
    }
    map_set_rectangular_obstacle(map.crater[0], 650, 540, 240, 260, robot_size);
    map_set_rectangular_obstacle(map.crater[1], 2350, 540, 240, 260, robot_size);
    map_set_rectangular_obstacle(map.crater[2], 1070, 1870, 240, 260, robot_size);
    map_set_rectangular_obstacle(map.crater[3], 1930, 1870, 240, 260, robot_size);

    /* Add fences */
    for (int i = 0; i < MAP_NUM_FENCE; i++) {
        map.fence[i] = oa_new_poly(4);
    }
    map_set_rectangular_obstacle(map.fence[0], 355, 370, 710, 22, robot_size);
    map_set_rectangular_obstacle(map.fence[1], 2645, 370, 710, 22, robot_size);

    /* Add opponent obstacle as points at origin */
    for (int i = 0; i < MAP_NUM_OPPONENT; i++) {
        map.opponents[i] = oa_new_poly(MAP_NUM_OPPONENT_EDGES);
        map_set_rectangular_obstacle(map.opponents[i], 0, 0, 0, 0, 0);
    }

    map.last_opponent_index = 0;
}

void map_set_opponent_obstacle(int index, int32_t x, int32_t y, int32_t opponent_size, int32_t robot_size)
{
    map_set_rectangular_obstacle(map.opponents[index], x, y, opponent_size, opponent_size, robot_size);
}

poly_t* map_get_opponent_obstacle(int index)
{
    return map.opponents[index];
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

void map_update_opponent_obstacle(int32_t x, int32_t y, int32_t opponent_size, int32_t robot_size)
{
    map_set_opponent_obstacle(map.last_opponent_index, x, y, opponent_size, robot_size);
    map.last_opponent_index++;
    if (map.last_opponent_index >= MAP_NUM_OPPONENT) {
        map.last_opponent_index = 0;
    }
}
