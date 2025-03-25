#ifndef CONTROL_TASK_H
#define CONTROL_TASK_H

#include "robotic_definitions.h"

// User Commands
void setEnd(UserCommand* cmd);
void setEndSpeed(UserCommand* cmd);
void setMotorAngles(UserCommand* cmd);
void setMotorSpeed(UserCommand* cmd);
void sleep(UserCommand* cmd);


// Task Definition
void controlTask(void *pvParameter);

// Helper Functions
void primeMotor(int motor, float desiredAngle, float speed);
void enableMotors();
void waitTilMotorsIdle();


#endif