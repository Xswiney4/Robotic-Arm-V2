#ifndef MOTOR_MONITOR_TASK_H
#define MOTOR_MONITOR_TASK_H

#include "robotic_definitions.h"
#include "motorModule.h"

class MotorMonitorTask{
    private:
        // Initialization
        bool isInitialized = false;

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
        void init(MotorModule** motors);


};

#endif