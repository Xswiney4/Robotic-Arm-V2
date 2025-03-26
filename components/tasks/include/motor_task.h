#ifndef MOTOR_TASK_H
#define MOTOR_TASK_H

#include "robotic_definitions.h"
#include "pca9548a.h"
#include "stepper.h"
#include "as5600.h"
#include "driver/gpio.h"


// Task Definition
void motorTask(void *pvParameter);

// Motor Functions
void setMotorAngle(MotorParams* params);

// Other Utils


#endif