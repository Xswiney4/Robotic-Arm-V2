#ifndef KINEMATICS_TASK_H
#define KINEMATICS_TASK_H

#include "robotFunctions.h"

class KinematicsTask{
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
        void kinematicsTask();

    public:

        // Task Controls
        void start();
        void stop();
        void restart();

        // Virtual Motor Angles
        float virtAngle[6];

        // RTOS Resources
        RtosResources* getRtosResources();

        // Constructor/Destructor
        KinematicsTask();
        ~KinematicsTask();

        // Initialization
        void init(RtosResources* resources, MotorModule** motors);

        

};

#endif