#ifndef MOTOR_TASK_H
#define MOTOR_TASK_H

#include "robotic_definitions.h"
#include "pca9548a.h"
#include "driver/gpio.h"


// Task Definition
void motorTask(void *pvParameter);
void stepMonitorTask(void *pvParameter);

// Motor Functions


// Motor Helper Functions

#endif