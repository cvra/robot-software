#ifndef ARM_TRAJECTORY_MANAGER_H
#define ARM_TRAJECTORY_MANAGER_H

#ifdef __cplusplus
extern "C" {
#endif

#include "blocking_detection_manager/blocking_detection_manager.h"

#include "scara/scara_waypoint.h"
#include "scara/scara.h"

#define ARM_TRAJ_MANAGER_FREQUENCY 20

enum arm_traj_state {
    ARM_READY=0,    /** Done, waiting for new goal */
    ARM_MOVING,     /** Moving towards goal */
    ARM_BLOCKED_XY, /** Blocked in xy plane, can't reach target position */
    ARM_BLOCKED_Z, /** Blocked in z axis, can't reach target position */
};

struct arm_traj_manager {
    enum arm_traj_state state;  /** Status of the arm, either following a
                                    trajectory or waiting for the next goal. */
    position_3d_t tol_mm;       /** Tolerance on x/y/z position in mm */

    struct blocking_detection blocking_detection_manager_xy; // Blocking detection in xy plane
    struct blocking_detection blocking_detection_manager_z;  // Blocking detection in z axis
};

extern struct arm_traj_manager main_arm_traj_manager;

void arm_traj_manager_init(struct arm_traj_manager* manager);
void arm_traj_manager_set_tolerances(struct arm_traj_manager *manager,
                                     float x_tol_mm, float y_tol_mm,
                                     float z_tol_mm);
void arm_traj_manager_set_blocking_detection_xy(struct arm_traj_manager *manager, int error_threshold_mm,
                                                int error_count_threshold);
void arm_traj_manager_set_blocking_detection_z(struct arm_traj_manager *manager, int error_threshold_mm,
                                               int error_count_threshold);
void arm_traj_wait_for_end(void);

void arm_trajectory_manager_start(scara_t* arm);

#ifdef __cplusplus
}
#endif

#endif /* ARM_TRAJECTORY_MANAGER_H */
