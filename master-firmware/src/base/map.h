#ifndef MAP_H
#define MAP_H

#ifdef __cplusplus
extern "C" {
#endif

#include "obstacle_avoidance/obstacle_avoidance.h"
#include <ch.h>

#define MAP_SIZE_X_MM 3000
#define MAP_SIZE_Y_MM 2000
#define MAP_NUM_OPPONENT 2
#define MAP_NUM_OPPONENT_EDGES 4

#define MAP_NUM_BLOCKS_CUBE 6
#define MAP_NUM_BLOCKS_CUBE_EDGES 8

#define MAP_NUM_WATER_DISPENSER 4
#define MAP_NUM_WATER_DISPENSER_EDGES 4

struct _map {
  poly_t *blocks_cube[MAP_NUM_BLOCKS_CUBE];
  poly_t *water_dispenser[MAP_NUM_WATER_DISPENSER];
  poly_t *wastewater_obstacle;

  poly_t *opponents[MAP_NUM_OPPONENT];
  uint8_t last_opponent_index;

  mutex_t lock;
};

/** Initialize the map of the Eurobot table with the static obstacles and
 * opponents
 */
void map_init(struct _map *map, int robot_size);

/** Set the position of the opponent identified by its index
 */
void map_set_opponent_obstacle(struct _map *map, int index, int32_t x,
                               int32_t y, int32_t opponent_size,
                               int32_t robot_size);

/** Get the position of the opponent identified by its index
 */
poly_t *map_get_opponent_obstacle(struct _map *map, int index);

/** Update opponent obstacle position and size
 */
void map_update_opponent_obstacle(struct _map *map, int32_t x, int32_t y,
                                  int32_t opponent_size, int32_t robot_size);

/** Set the points of a rectangle given its center position and size
 */
void map_set_rectangular_obstacle(poly_t *opponent, int center_x, int center_y,
                                  int size_x, int size_y, int robot_size);

poly_t *map_get_cubes_obstacle(struct _map *map, int index);
void map_set_cubes_obstacle(struct _map *map, int index, int x, int y,
                            int robot_size);

#ifdef __cplusplus
}
#endif

#endif /* MAP_H */
