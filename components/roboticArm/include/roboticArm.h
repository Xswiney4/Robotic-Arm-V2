#ifndef ROBOTIC_ARM_H
#define ROBOTIC_ARM_H

#include "pca9548a.h"
#include "motorModule.h"
#include "robotic_definitions.h"

// Tasks Definitions
#include "communication_task.h"
#include "control_task.h"
#include "kinematics_task.h"
#include "motor_task.h"
#include "motor_monitor_task.h"

class RoboticArm{

    private:

        // Variables
        PCA9548A pca9548a;      // PCA9548A object
        MotorModule* motors[6] = {nullptr}; // Pointers to all my motor objects

        // Tasks
        MotorTask motorTask[6];
        MotorMonitorTask motorMonitorTask;
        ControlTask controlTask;
        KinematicsTask kinematicsTask;
        CommunicationTask communicationTask;

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

        bool initMotorMonitor();    // Initializes the motor task monitor

        // Robot Task Call
        void runTask(const char* name, void* args);
        
        // Sends a user command to the central command task
        void sendUserCommand(UserCommand* cmd);

    public:

        // Constructer/Destructor
        RoboticArm();
        ~RoboticArm();

        // User Commands
        void setEnd(float x, float y, float z, float pitch, float yaw, float roll);
        void setEndSpeed(float speed);
        void setMotorAngles(float angle1, float angle2, float angle3, float angle4, float angle5, float angle6);
        void setMotorSpeed(int motor, float speed);
        void sleep(int ms);

};

#endif