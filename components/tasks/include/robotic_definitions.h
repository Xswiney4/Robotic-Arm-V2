#ifndef ROBOTIC_DEFINITIONS_H
#define ROBOTIC_DEFINITIONS_H

// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// ~~ Libraries ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "freertos/queue.h"

// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// ~~ Structures ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

// Input User Command Structure
struct UserCommand{
    int commandNum = -1;
    const char* name = "NULL";
    double params[6] = {0,0,0,0,0,0};
};


// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// ~~ FreeRTOS ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

// Queues
extern QueueHandle_t userCmdRaw;  // Queue for raw user commands (Character Array)
extern QueueHandle_t userCmd;  // Queue for user commands (UserCommand Struct)

extern QueueHandle_t j1sDesiredAngleQueue;  // Queue for stepper motor angle
extern QueueHandle_t j1sParamsQueue;  // Queue for stepper motor params
extern QueueHandle_t j2sDesiredAngleQueue;  // Queue for stepper motor angle
extern QueueHandle_t j2sParamsQueue;  // Queue for stepper motor params
extern QueueHandle_t j3sDesiredAngleQueue;  // Queue for stepper motor angle
extern QueueHandle_t j3sParamsQueue;  // Queue for stepper motor params
extern QueueHandle_t j4sDesiredAngleQueue;  // Queue for stepper motor angle
extern QueueHandle_t j4sParamsQueue;  // Queue for stepper motor params
extern QueueHandle_t j5sDesiredAngleQueue;  // Queue for stepper motor angle
extern QueueHandle_t j5sParamsQueue;  // Queue for stepper motor params
extern QueueHandle_t j6sDesiredAngleQueue;  // Queue for stepper motor angle
extern QueueHandle_t j6sParamsQueue;  // Queue for stepper motor params

// Task Notification
extern TaskHandle_t KinematicsSolved; // Flags if Kinematics Solver is idle

// Event Groups
extern EventGroupHandle_t motorEnable; // Enables Motors
extern EventGroupHandle_t motorIdle;   // Flags if motor is idle
extern EventGroupHandle_t motorReady;  // Signals motor is ready

#endif