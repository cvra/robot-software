#include "robot_helpers/math_helpers.h"
#include "base_controller.h"
#include "map.h"

static struct _map map;


void map_init(int robot_size)
{
    // Initialise obstacle avoidance state
    oa_init();

    /* Define table borders */
    polygon_set_boundingbox(robot_size/2, robot_size/2,
                            MAP_SIZE_X_MM - robot_size/2, MAP_SIZE_Y_MM - robot_size/2);

    /* Add obstacles */
    map.crater = oa_new_poly(4);
    map_set_rectangular_obstacle(map.crater, 650, 540, 240, 260, robot_size);

    map.fence = oa_new_poly(4);
    map_set_rectangular_obstacle(map.fence, 355, 370, 710, 22, robot_size);

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
    opponent->pts[0].x = math_clamp_value(center_x + (size_x + robot_size) / 2, 0, MAP_SIZE_X_MM);
    opponent->pts[0].y = math_clamp_value(center_y - (size_y + robot_size) / 2, 0, MAP_SIZE_Y_MM);

    opponent->pts[1].x = math_clamp_value(center_x + (size_x + robot_size) / 2, 0, MAP_SIZE_X_MM);
    opponent->pts[1].y = math_clamp_value(center_y + (size_y + robot_size) / 2, 0, MAP_SIZE_Y_MM);

    opponent->pts[2].x = math_clamp_value(center_x - (size_x + robot_size) / 2, 0, MAP_SIZE_X_MM);
    opponent->pts[2].y = math_clamp_value(center_y + (size_y + robot_size) / 2, 0, MAP_SIZE_Y_MM);

    opponent->pts[3].x = math_clamp_value(center_x - (size_x + robot_size) / 2, 0, MAP_SIZE_X_MM);
    opponent->pts[3].y = math_clamp_value(center_y - (size_y + robot_size) / 2, 0, MAP_SIZE_Y_MM);
}

void map_update_opponent_obstacle(int32_t x, int32_t y, int32_t opponent_size, int32_t robot_size)
{
    map_set_opponent_obstacle(map.last_opponent_index, x, y, opponent_size, robot_size);
    map.last_opponent_index++;
    if (map.last_opponent_index >= MAP_NUM_OPPONENT) {
        map.last_opponent_index = 0;
    }
}
