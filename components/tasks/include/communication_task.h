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
    {"sleep", 20, 1},
};
const int numCommands = sizeof(commands) / sizeof(commands[0]);  // Get array size dynamically

class CommunicationTask{
    private:

        // Initialization
        void initBluetooth();
        void initUART();

        // Task Definition
        static void taskEntry(void* pvParameters);
        void communicationTask();

        // Task Handle
        TaskHandle_t taskHandle = nullptr;

        // Handlers
        UserCommand userCmdParser(char *buffer);

        // Other Utils
        void exportUserCommand(UserCommand* cmd);

    public:

        // Task Controls
        void start();
        void stop();
        void restart();

        // Constructor/Destructor
        CommunicationTask();
        ~CommunicationTask();

};

#endif