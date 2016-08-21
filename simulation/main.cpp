#include <iostream>
#include <unistd.h>

extern "C" {
#include "math/geometry/polygon.h"
#include "obstacle_avoidance/obstacle_avoidance.h"
}

int main(int argc, const char** argv)
{
    (void) argc;
    (void) argv;

    /* Set table */
    const int robot_size = 150;
    polygon_set_boundingbox(robot_size, robot_size, 3000-robot_size, 2000-robot_size);

    oa_init();
    oa_reset();

    while (true) {
        std::cout << "Running..."<< std::endl;
        usleep(100 * 1000);
    }
}
