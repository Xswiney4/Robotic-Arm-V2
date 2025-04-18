#ifndef KINEMATICS_TASK_H
#define KINEMATICS_TASK_H

#include "robotic_definitions.h"

// User Commands
void setEndKinCalc(UserCommand* cmd, MotorModule** motors, float* virtMotorAngle);
void setMotorAnglesKinCalc(UserCommand* cmd, MotorModule** motors, float* virtMotorAngle);

// Task Definition
void kinematicsTask(void *pvParameter);

// Other Utils

#endif