#include "joint.h"

void joint_set_callbacks(joint_t *joint, void (*set_position)(void *, float),
                         void (*set_velocity)(void *, float),
                         float (*get_position)(void *), void *args) {
  joint->set_position = set_position;
  joint->set_velocity = set_velocity;
  joint->get_position = get_position;
  joint->args = args;
}
