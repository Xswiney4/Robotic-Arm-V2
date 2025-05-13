#ifndef MOTOR_TASK_H
#define MOTOR_TASK_H

#include "robotFunctions.h"
#include "motorModule.h"

class MotorTask{
    private:

        // Initialization
        bool isInitialized = false;

        // RTOS Resources
        RtosResources* rtosResources;
        
        // Task Handle
        TaskHandle_t taskHandle = nullptr;
        const char* taskName;

        // Motors
        MotorModule* motor;

        // Motor Task Definition
        static void taskEntry(void* pvParameters);
        void motorTask();

    public:

        // Task Controls
        void start();
        void stop();
        void restart();

        // RTOS Resources
        RtosResources* getRtosResources();

        // Constructor/Destructor
        MotorTask();
        ~MotorTask();

        // Initialization
        void init(RtosResources* resources, const char* taskName, MotorModule* motor);


};

#endif