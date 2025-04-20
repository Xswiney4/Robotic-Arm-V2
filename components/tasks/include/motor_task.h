#ifndef MOTOR_TASK_H
#define MOTOR_TASK_H

#include "robotic_definitions.h"
#include "motorModule.h"

class MotorTask{
    private:

        // Initialization
        bool isInitialized = false;
        
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

        // Constructor/Destructor
        MotorTask();
        ~MotorTask();

        // Initialization
        void init(const char* taskName, MotorModule* motor);


};

#endif