#include "scara_inverse_kinematics_controller.h"

#include <scara/scara_jacobian.h>
#include <scara/scara_kinematics.h>

#include <string.h>

void scara_ik_controller_init(scara_ik_controller_t* controller)
{
    memset(controller, 0, sizeof(scara_ik_controller_t));

    pid_init(&controller->x_pid);
    pid_init(&controller->y_pid);
}

void scara_ik_controller_set_geometry(scara_ik_controller_t *controller,
                                      float *length)
{
    controller->length[0] = length[0];
    controller->length[1] = length[1];
}

scara_joint_setpoints_t
scara_ik_controller_process(scara_ik_controller_t *controller,
                            position_3d_t desired,
                            scara_joint_positions_t measured)
{
  point_t pos = scara_forward_kinematics(measured.shoulder, measured.elbow,
                                         controller->length);

  float consign_x = pid_process(&controller->x_pid, pos.x - desired.x);
  float consign_y = pid_process(&controller->y_pid, pos.y - desired.y);

  float velocity_alpha, velocity_beta;
  scara_jacobian_compute(consign_x, consign_y, measured.shoulder,
                         measured.elbow, controller->length[0],
                         controller->length[1], &velocity_alpha,
                         &velocity_beta);

  scara_joint_setpoints_t joint_setpoints = {
      .z = {POSITION, desired.z},
      .shoulder = {VELOCITY, velocity_alpha},
      .elbow = {VELOCITY, velocity_beta}};

  return joint_setpoints;
}
