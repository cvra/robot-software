#ifndef HAND_UTILS_H
#define HAND_UTILS_H

#ifdef __cplusplus
extern "C" {
#endif

#include <aversive/math/vect2/vect2.h>
#include <aversive/math/geometry/polygon.h>

typedef enum {
    HAND_COORDINATE_HAND=0, /**< Coordinate relative to the hand base. */
    HAND_COORDINATE_ARM,    /**< Coordinate relative to the arm base. */
    HAND_COORDINATE_ROBOT,  /**< Coordinate relative to the robot center. */
    HAND_COORDINATE_TABLE   /**< Coordinate relative to the table (absolute). */
} hand_coordinate_t;

typedef enum {
    FINGER_CLOSED=0,
    FINGER_OPEN,
    FINGER_RETRACTED,
} finger_state_t;


float hand_heading_arm2hand(float arm_angle, float arm_offset);
float hand_heading_hand2arm(float hand_angle, float arm_offset);

float hand_heading_robot2arm(float robot_angle, float offset_angle);
float hand_heading_arm2robot(float arm_angle, float offset_angle);

float hand_heading_table2robot(float table_angle, float robot_a_rad);
float hand_heading_robot2table(float robot_angle, float robot_a_rad);

#ifdef __cplusplus
}
#endif

#endif /* HAND_UTILS_H */
