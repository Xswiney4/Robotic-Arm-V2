#ifndef CONTROL_TASK_H
#define CONTROL_TASK_H

#include "robotic_definitions.h"

// Structure Format
struct Command{
    const char* name;
    int commandNum;
    int numParams;
};

// User Commands List:
const Command commands[] = {
    {"setEnd", 0, 6},
    {"setEndSpeed", 1, 1},
    {"setMotorAngle", 10, 2},
    {"setMotorSpeed", 11, 2},
};
const int numCommands = sizeof(commands) / sizeof(commands[0]);  // Get array size dynamically

// User Commands
void setEnd(double x, double y, double z, double pitch, double yaw, double roll);
void setEndSpeed(double speed);
void setMotorAngle(int motor, double angle);
void setMotorSpeed(int motor, double speed);

// Handlers
UserCommand userCmdDecoder(char *buffer); // Decodes a character string and returns a user Command structure

// Task Definition
void controlTask(void *pvParameter);

// Other Utils


#endif