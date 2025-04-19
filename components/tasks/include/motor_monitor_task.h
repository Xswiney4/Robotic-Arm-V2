#ifndef MOTOR_MONITOR_TASK_H
#define MOTOR_MONITOR_TASK_H

#include "robotic_definitions.h"
#include "motorModule.h"

class MotorMonitorTask{
    private:
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
        MotorMonitorTask(MotorModule** motors);
        ~MotorMonitorTask();


};

#endif