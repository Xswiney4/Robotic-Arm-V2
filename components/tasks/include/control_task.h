#ifndef CONTROL_TASK_H
#define CONTROL_TASK_H

#include "robotFunctions.h"

class ControlTask{
    private:

        // Initialization
        bool isInitialized = false;

        // RTOS Resources
        RtosResources* rtosResources;

        // Task Handle
        TaskHandle_t taskHandle = nullptr;

        // Task Definition
        static void taskEntry(void* pvParameters);
        void controlTask();

    public:

        // Task Controls
        void start();
        void stop();
        void restart();

        // RTOS Resources
        RtosResources* getRtosResources();

        // Constructor/Destructor
        ControlTask();
        ~ControlTask();

        // Initialization
        void init(RtosResources* resources);

};

#endif