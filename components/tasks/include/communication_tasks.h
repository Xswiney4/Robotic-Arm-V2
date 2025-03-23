#ifndef COMMUNICATION_TASKS_H
#define COMMUNICATION_TASKS_H

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
};
const int numCommands = sizeof(commands) / sizeof(commands[0]);  // Get array size dynamically

// Initializations
void initBluetooth();
void initUART();

// Handlers
UserCommand userCmdParser(char *buffer);

// Task Definition
void communicationsTask(void *pvParameter);

// Other Utils

#endif