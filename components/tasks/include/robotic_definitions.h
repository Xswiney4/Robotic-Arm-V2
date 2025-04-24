#ifndef ROBOTIC_DEFINITIONS_H
#define ROBOTIC_DEFINITIONS_H

// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// ~~ Libraries ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "freertos/queue.h"

#include "as5600.h"

// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// ~~ Structures ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

// Input User Command Structure
struct UserCommand{
    int commandNum = -1;
    const char* name = "NULL";
    float params[6] = {0,0,0,0,0,0};
};

// Structure Format
struct Command{
    const char* name;
    int commandNum;
    int numParams;
};

// User Commands List, in the format of:
// {name, commandNum, numParams}
const Command commands[] = {
    {"setEnd", 0, 6},
    {"setEndSpeed", 1, 1},
    {"setMotorAngle", 10, 2},
    {"setMotorSpeed", 11, 2},
    {"sleep", 20, 1},
};
const int numCommands = sizeof(commands) / sizeof(commands[0]);  // Get array size dynamically

// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// ~~ FreeRTOS Resources ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

// Queues
extern QueueHandle_t controlCmd;       // Queue for user commands      (UserCommand Struct)
extern QueueHandle_t kinematicsCmd;    // Queue for kinematics task    (UserCommand Struct)

extern QueueHandle_t motorTargetsQueue[6];

// Task Notification
extern TaskHandle_t kinematicsSolved; // Flags if Kinematics Solver is idle

// Event Groups
extern EventGroupHandle_t motorEnabled; // Enables Motors
extern EventGroupHandle_t motorIdle;   // Flags if motor is idle
extern EventGroupHandle_t motorReady;  // Signals motor is ready

#endif