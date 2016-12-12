#include "base_controller.h"
#include "map.h"

static struct _map map;

static const int MAP_SIZE_X_MM = 3000;
static const int MAP_SIZE_Y_MM = 2000;

void map_init(int robot_size)
{
    // Initialise obstacle avoidance state
    oa_init();

    /* Define table borders */
    polygon_set_boundingbox(robot_size/2, robot_size/2,
                            MAP_SIZE_X_MM - robot_size/2,
                            MAP_SIZE_Y_MM - robot_size/2);

    /* Add obstacles */
    map.crater = oa_new_poly(4);
    map_set_rectangular_obstacle(map.crater, 650, 540, 240, 260, robot_size);

    map.fence = oa_new_poly(4);
    map_set_rectangular_obstacle(map.fence, 355, 370, 710, 22, robot_size);
}

void map_set_rectangular_obstacle(poly_t* opponent, int center_x, int center_y, int size_x, int size_y, int robot_size)
{
    opponent->pts[0].x = map_clamp_value(center_x + (size_x + robot_size) / 2, 0, MAP_SIZE_X_MM);
    opponent->pts[0].y = map_clamp_value(center_y - (size_y + robot_size) / 2, 0, MAP_SIZE_Y_MM);

    opponent->pts[1].x = map_clamp_value(center_x + (size_x + robot_size) / 2, 0, MAP_SIZE_X_MM);
    opponent->pts[1].y = map_clamp_value(center_y + (size_y + robot_size) / 2, 0, MAP_SIZE_Y_MM);

    opponent->pts[2].x = map_clamp_value(center_x - (size_x + robot_size) / 2, 0, MAP_SIZE_X_MM);
    opponent->pts[2].y = map_clamp_value(center_y + (size_y + robot_size) / 2, 0, MAP_SIZE_Y_MM);

    opponent->pts[3].x = map_clamp_value(center_x - (size_x + robot_size) / 2, 0, MAP_SIZE_X_MM);
    opponent->pts[3].y = map_clamp_value(center_y - (size_y + robot_size) / 2, 0, MAP_SIZE_Y_MM);
}

int map_clamp_value(int value, int min, int max)
{
    if (value < min) {
        return min;
    } else if (value > max) {
        return max;
    } else {
        return value;
    }
}
