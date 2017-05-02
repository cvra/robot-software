#ifndef SCARA_WAYPOINT_H
#define SCARA_WAYPOINT_H

#include <stdint.h>

/* Maximum number of waypoints in a trajectory */
#define SCARA_TRAJ_MAX_NUM_FRAMES 10

/** This typedef stores the type of the coordinate system used for a trajectory. */
typedef enum {
    COORDINATE_ARM=0, /**< Coordinate relative to the arm shoulder. */
    COORDINATE_ROBOT, /**< Coordinate relative to the robot center. */
    COORDINATE_TABLE  /**< Coordinate relative to the table (absolute). */
} scara_coordinate_t;

/** This structure holds the data for a single waypoint of an arm trajectory (a waypoint). */
typedef struct {
    int32_t date;                       /**< waypoint validity date, since the boot of the robot in us. */
    float position[3];                  /**< Position of the arm at the data. */
    scara_coordinate_t coordinate_type; /**< The coordinate system of this trajectory. */
    float length[3];                    /**< The length of the arm to use. */
    float hand_angle;                   /**< The angle of the hand in degree. */
} scara_waypoint_t;


/** This structure holds the data of a single arm trajectory. */
typedef struct {
    scara_waypoint_t frames[SCARA_TRAJ_MAX_NUM_FRAMES]; /**< Trajectory waypoints. */
    int frame_count;          /**< Number of frames. */
} scara_trajectory_t;

#endif /* SCARA_WAYPOINT_H */
