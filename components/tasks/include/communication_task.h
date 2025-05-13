#ifndef COMMUNICATION_TASK_H
#define COMMUNICATION_TASK_H

#include "robotFunctions.h"

class CommunicationTask{
    private:

        // Initialization
        bool isInitialized = false;
        void initBluetooth();
        void initUART();

        // RTOS Resources
        RtosResources* rtosResources;

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

        // Initialization
        void init(RtosResources* resources);

};

#endif