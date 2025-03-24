#ifndef KINEMATICS_TASK_H
#define KINEMATICS_TASK_H

#include "robotic_definitions.h"

// User Commands
void setEndKinCalculation(UserCommand* cmd);

// Task Definition
void kinematicsTask(void *pvParameter);

// Other Utils

#endif