#ifndef ARMS_CONTROLLER_H
#define ARMS_CONTROLLER_H

#include "scara/scara.h"

#define ARMS_FREQUENCY 10

extern scara_t left_arm;
extern scara_t right_arm;

void arms_init(void);

void arms_controller_start(void);

#endif
