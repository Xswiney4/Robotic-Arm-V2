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
    float params[6] = {0,0,0,0,0,0};
};


// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// ~~ FreeRTOS ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

// Queues
extern QueueHandle_t controlCmd;       // Queue for user commands      (UserCommand Struct)
extern QueueHandle_t kinematicsCmd; // Queue for kinematics task    (UserCommand Struct)

extern QueueHandle_t desiredAngleQueue[6];
extern QueueHandle_t paramsQueue[6];

// Task Notification
extern TaskHandle_t KinematicsSolved; // Flags if Kinematics Solver is idle

// Event Groups
extern EventGroupHandle_t motorEnable; // Enables Motors
extern EventGroupHandle_t motorIdle;   // Flags if motor is idle
extern EventGroupHandle_t motorReady;  // Signals motor is ready

#endif