#ifndef MOTOR_TASK_H
#define MOTOR_TASK_H

#include "robotic_definitions.h"
#include "pca9548a.h"
#include "stepper.h"
#include "as5600.h"
#include "driver/gpio.h"

struct MotorParams{

    // Objects
    StepperMotor* stepper;
    AS5600* as5600;

    // Motor Parameters
    float speed; // In degrees/second

    // FreeRTOS
    uint8_t eventGroupBit;
    QueueHandle_t desiredAngleQueueHandle;
    QueueHandle_t paramsQueueHandle;
    
};

// Initialization
void initMotors(); // Sets up motor control tasks

// Task Definition
void motorTask(void *pvParameter);

// Other Utils


#endif