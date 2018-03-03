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

/** Position in 3D space */
typedef struct {
    float x;
    float y;
    float z;
} position_3d_t;
typedef position_3d_t velocity_3d_t;

/** This structure holds the data for a single waypoint of an arm trajectory (a waypoint). */
typedef struct {
    int32_t date;                       /**< waypoint validity date, since the boot of the robot in us. */
    position_3d_t position;             /**< Position of the arm at the date. */
    velocity_3d_t max_velocity;         /**< Maximum velocity of the arm during trajectory segment. */
    scara_coordinate_t coordinate_type; /**< The coordinate system of this trajectory. */
    float length[2];                    /**< The length of the arm to use. */
} scara_waypoint_t;


/** This structure holds the data of a single arm trajectory. */
typedef struct {
    scara_waypoint_t frames[SCARA_TRAJ_MAX_NUM_FRAMES]; /**< Trajectory waypoints. */
    int frame_count;          /**< Number of frames. */
} scara_trajectory_t;

#endif /* SCARA_WAYPOINT_H */
