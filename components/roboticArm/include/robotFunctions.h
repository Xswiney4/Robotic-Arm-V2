#ifndef ROBOT_FUNCITONS_H
#define ROBOT_FUNCTIONS_H

// #include "robotic_definitions.h"
#include <functional>
#include <array>

// FreeRTOS
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "freertos/queue.h"

// Argument Types
#define ARGTYPE_NULL           -1
#define ARGTYPE_INT             1
#define ARGTYPE_FLOAT           2
#define ARGTYPE_POSE            3
#define ARGTYPE_MOTOR_TARGET    4



// ~~ Robot Task Definitions ~~

// setEnd(pos, ori)
void setEndSM(RtosResources* rtosResources, void* context, void* args);
void setEndCalc(RtosResources* rtosResources, void* context, void* args);


// setAngles({angle, speeed}[6]);
void setAnglesSM(RtosResources* rtosResources, void* context, void* args);
void setAnglesCalc(RtosResources* rtosResources, void* context, void* args);

// sleep(ms)
void sleepSM(RtosResources* rtosResources, void* context, void* args);

// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

struct RtosResources{
    QueueHandle_t controlCmd;       // Queue for user commands      (UserCommand Struct)
    QueueHandle_t kinematicsCmd;    // Queue for kinematics task    (UserCommand Struct)

    QueueHandle_t motorTargetsQueue[6];

    // Task Notification
    TaskHandle_t kinematicsSolved; // Flags if Kinematics Solver is idle

    // Event Groups
    EventGroupHandle_t motorEnabled; // Enables Motors
    EventGroupHandle_t motorIdle;   // Flags if motor is idle
    EventGroupHandle_t motorReady;  // Signals motor is ready
};

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
    float angle = -1.0f;
    float speed = -1.0f;
    float startAngle;
};

// Robot Task Structure
struct RobotTask{
    // Readable Name
    const char* name;

    // Callbacks for control and calculation tasks
    std::function<void(RtosResources*, void*, void*)> stateMachineFunc;  // Runs at higher priority and if necessary, waits for information from calculationFunc
    std::function<void(RtosResources*, void*, void*)> calculationFunc;   // Runs at lowest priority and sends information to stateMachineFunc

    // Communication Task Parsing
    int argType = ARGTYPE_NULL;     // Type of argument the callbacks input
    int argCount = -1;              // Amount of arguments callbacks input (argCount = 6 if input is an array of 6)

};

// Robot Tasks that are registered into the system
const RobotTask robotTasks[] = {
    {"setEnd", setEndSM, setEndCalc, ARGTYPE_POSE, 1},
    {"setAngles", setAnglesSM, setAnglesCalc, ARGTYPE_MOTOR_TARGET, 6},
    {"sleep", sleepSM, nullptr, ARGTYPE_INT, 1},
};

const int numRobotTasks = sizeof(robotTasks) / sizeof(robotTasks[0]);  // Get array size dynamically

#endif