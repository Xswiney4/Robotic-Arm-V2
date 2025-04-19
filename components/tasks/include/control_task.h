#ifndef CONTROL_TASK_H
#define CONTROL_TASK_H

#include "robotic_definitions.h"

class ControlTask{
    private:

        // Task Handle
        TaskHandle_t taskHandle = nullptr;

        // Task Definition
        static void taskEntry(void* pvParameters);
        void controlTask();

        // User Commands
        void setEnd(UserCommand* cmd);
        void setEndSpeed(UserCommand* cmd);
        void setMotorAngles(UserCommand* cmd);
        void setMotorSpeed(UserCommand* cmd);
        void sleep(UserCommand* cmd);

        // Other Utils
        void primeMotor(int motor, float desiredAngle, float speed);
        void enableMotors();
        void waitTilMotorsIdle();

    public:

        // Task Controls
        void start();
        void stop();
        void restart();

        // Constructor/Destructor
        ControlTask();
        ~ControlTask();

};

#endif