#ifndef ROBOTIC_ARM_H
#define ROBOTIC_ARM_H

#include "pca9548a.h"
#include "motor_task.h"
#include "robotic_definitions.h"

class RoboticArm{

    private:

        // Variables
        PCA9548A pca9548a;     // PCA9548A object

        // Initializations (Returns true if error)
        bool initAll();             // Runs all initializations

        bool initRTOSComms();       // Configures FreeRTOS queues/semaphores/event groups
        bool errorCheckComms();     // Checks that all FreeRTOS Comms are initialized

        bool initCommunications();  // Initializes communications task
        bool initControl();         // Initializes central control task
        bool initKinematics();      // Initializes kinematics calculations task
        bool initAllMotors();       // Initializes all motor tasks
        
        bool initMotor1();      // Initializes motor task 1
        bool initMotor2();      // Initializes motor task 2
        bool initMotor3();      // Initializes motor task 3
        bool initMotor4();      // Initializes motor task 4
        bool initMotor5();      // Initializes motor task 5
        bool initMotor6();      // Initializes motor task 6

        // Sends a user command to the central command task
        void sendUserCommand(UserCommand* cmd);

    public:

        // Constructer/Destructor
        RoboticArm();
        ~RoboticArm();

        // User Commands
        void setEnd(float x, float y, float z, float pitch, float yaw, float roll);
        void setEndSpeed(float speed);
        void setMotorAngle(int motor, float angle);
        void setMotorSpeed(int motor, float speed);

};

#endif