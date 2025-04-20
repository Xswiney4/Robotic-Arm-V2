#ifndef KINEMATICS_TASK_H
#define KINEMATICS_TASK_H

#include "robotic_definitions.h"

class KinematicsTask{
    private:

        // Initialization
        bool isInitialized = false;

        // Task Handle
        TaskHandle_t taskHandle = nullptr;

        // Motors
        MotorModule** motors;

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
        void init(MotorModule** motors);

        

};

#endif