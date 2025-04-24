#ifndef ROBOT_FUNCITONS_H
#define ROBOT_FUNCTIONS_H

// #include "robotic_definitions.h"
#include <functional>
#include <array>

// Argument Types
#define ARGTYPE_NULL           -1
#define ARGTYPE_POSE            1
#define ARGTYPE_MOTOR_TARGET    2



// ~~ Robot Task Definitions ~~

// setEnd(pos, ori)
void setEndSM(void* args);
void setEndCalc(void* args);


// setAngles({angle, speeed}[6]);
void setAnglesSM(void* args);
void setAnglesCalc(void* args);

// sleep(ms)
void sleepSM(void* args);

// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

// User Command Structure
struct UserCommand{
    const char* name;
    void* args = nullptr;
};

// Pose Structure
struct Pose{
    std::array<float, 3> position;
    std::array<float, 3> orientation;
};

// Motor Target Structure
struct MotorTarget{
    float angle;
    float speed;
};

// Robot Task Structure
struct RobotTask{
    // Readable Name
    const char* name;

    // Callbacks for control and calculation tasks
    std::function<void(void*)> stateMachineFunc;  // Runs at higher priority and if necessary, waits for information from calculationFunc
    std::function<void(void*)> calculationFunc;   // Runs at lowest priority and sends information to stateMachineFunc

    // Communication Task Parsing
    int argType = ARGTYPE_NULL;     // Type of argument the callbacks input
    int argCount = -1;              // Amount of arguments callbacks input (argCount = 6 if input is an array of 6)

};

// Robot Tasks that are registered into the system
const RobotTask robotTasks[] = {
    {"setEnd", setEndSM, setEndCalc},
    {"setAngles", setAnglesSM, setAnglesCalc},
    {"sleep", sleepSM, nullptr},
};

#endif