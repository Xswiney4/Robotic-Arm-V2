// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// ~~ Libraries ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

#include "config.h"
#include "robotFunctions.h"

// Tasks
#include "control_task.h"
#include "kinematics_task.h"

// Component Definitions
#include "motorModule.h"

// ESP/FreeRTOS
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

// Utils
#include <cmath>


// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// ~~ setEnd(pose) ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

/*
Sets the endpoint given a position (x, y, z) and orientation (pitch, yaw, roll) of the end effector
The control structure is as follows:
- UserCommand is sent over to Kinematics Solver to be solved
- Wait until all motors are ready AND Kinematic Solver flags 'solved'
- Enable all motors
- Break
*/

void setEndSM(void* context, void* args){

    // Variable Handling
    ControlTask* controlTask = static_cast<ControlTask*>(context);
    RtosResources* res = controlTask->getRtosResources();
    MotorTarget* targets = static_cast<MotorTarget*>(args);

}

void setEndCalc(void* context, void* args){

    // Variable Handling
    KinematicsTask* kinematicsTask = static_cast<KinematicsTask*>(context);
    RtosResources* res = kinematicsTask->getRtosResources();
    MotorTarget* targets = static_cast<MotorTarget*>(args);

}

// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// ~~ setAngles(motorTarget[6]) ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

/*
Sets the angle of a given motor in degrees, it uses STEPPER_SPEED as the speed for the motors
The structure is as follows:
- Angle is sent over to motor
- Wait until given motor is ready
- Enable given motor
- Break
*/

void setAnglesSM(void* context, void* args){

    ESP_LOGD(TASK_NAME_CONTROL, "Started setAnglesSM");

    // Variable Handling
    ControlTask* controlTask = static_cast<ControlTask*>(context);
    RtosResources* res = controlTask->getRtosResources();
    float* targets = static_cast<float*>(args);
    
    uint8_t motorBitMask = 0x00;

    // Loop going through each parameter input
    for(int i = 0; i < 6; i++){
        // If parameter is valid
        if(targets[i] != -1.0f){

            // Adds the motor to the global bit mask
            motorBitMask |= (1 << i);
        }
    }

    // Log
    ESP_LOGD(TASK_NAME_CONTROL, "Motor Bit Mask: 0x%2X", motorBitMask);

    // Wait for all motors to be ready (Once they recieve targets from the kinematics task)
    xEventGroupWaitBits(res->motorReady, motorBitMask, pdTRUE, pdTRUE, portMAX_DELAY);
    
    // Enable Motors
    xEventGroupClearBits(res->motorIdle, motorBitMask);
    xEventGroupSetBits(res->motorEnabled, motorBitMask);

    // Wait until all motor's are idle before continuing
    xEventGroupWaitBits(res->motorIdle, motorBitMask, pdFALSE, pdTRUE, portMAX_DELAY);

    // Control Task Cleanup
    delete[] targets;
}

// Converts a position and orientation into joint angles, and then sends them to the motor tasks
void setAnglesCalc(void* context, void* args){

    ESP_LOGD(TASK_NAME_KINEMATICS, "Started setAnglesCalc");

    // Variable Handling
    KinematicsTask* kinematicsTask = static_cast<KinematicsTask*>(context);
    RtosResources* res = kinematicsTask->getRtosResources();
    float* targets = static_cast<float*>(args);

    // Loop going through each parameter input
    for(int i = 0; i < 6; i++){
        // If parameter is valid
        if(targets[i] != -1.0f){

            TargetParams refinedTarget;

            // Difference the input motor needs to move in order for the output angle to reach the target
            float inputDiff = targets[i] - kinematicsTask->virtAngle[i];

            // Calculate the number of steps needed
            refinedTarget.numSteps = (int)roundf(inputDiff / DEG_PER_STEP[i]);
            
            
            // Calculates the step time in ms
            float stepPeriodMs = 1000.0f * DEG_PER_STEP[i] / STEPPER_SPEED;

            refinedTarget.xFrequency = pdMS_TO_TICKS(stepPeriodMs);
            refinedTarget.targetAngle = targets[i];


            // Send target to motor
            xQueueSend(res->motorTargetsQueue[i], &refinedTarget, portMAX_DELAY);

            // Set the virtMotorAngle and
            kinematicsTask->virtAngle[i] = targets[i];

        }
    }
}
// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// ~~ sleep(ms) ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

// Puts the controller to sleep for a given time in ms
void sleepSM(void* context, void* args){

    // Context isn't necessary

    // Cast args
    int timeMs = *static_cast<int*>(args);

    // Delay
    ESP_LOGD(TASK_NAME_CONTROL, "Delaying control task for %dms", timeMs);
    vTaskDelay(pdMS_TO_TICKS(timeMs));

    // Control Task Cleanup
    delete static_cast<int*>(args);
}