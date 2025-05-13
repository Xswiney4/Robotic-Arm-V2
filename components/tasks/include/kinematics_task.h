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
        float virtAngle[6];

        // Task Definition
        static void taskEntry(void* pvParameters);
        void kinematicsTask();

        // User Commands
        void setEndKinCalc(UserCommand* cmd, MotorModule** motors, float* virtMotorAngle);
        void setMotorAnglesKinCalc(UserCommand* cmd, MotorModule** motors, float* virtMotorAngle);

        // Other Utils

    public:

        // Task Controls
        void start();
        void stop();
        void restart();

        // Constructor/Destructor
        KinematicsTask();
        ~KinematicsTask();

        // Initialization
        void init(RtosResources* resources, MotorModule** motors);

        

};

#endif