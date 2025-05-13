#ifndef MOTOR_MONITOR_TASK_H
#define MOTOR_MONITOR_TASK_H

#include "robotFunctions.h"
#include "motorModule.h"

class MotorMonitorTask{
    private:
        // Initialization
        bool isInitialized = false;

        // RTOS Resources
        RtosResources* rtosResources;

        // Task Handle
        TaskHandle_t taskHandle = nullptr;

        // Motors
        MotorModule** motors;

        // Task Definition
        static void taskEntry(void* pvParameters);
        void stepMonitorTask();

    public:

        // Task Controls
        void start();
        void stop();
        void restart();

        // Constructor/Destructor
        MotorMonitorTask();
        ~MotorMonitorTask();

        // Initialization
        void init(RtosResources* resources, MotorModule** motors);


};

#endif