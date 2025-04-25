// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// ~~ Libraries ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

#include "config.h"
#include "robotFunctions.h"

// Component Definitions

// ESP/FreeRTOS
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"


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

void setAnglesSM(RtosResources* rtosResources, void* args){

    MotorTarget* targets = static_cast<MotorTarget*>(args);
    
    uint8_t motorBitMask = 0x00;

    // Loop going through each parameter input
    for(int i = 0; i < 6; i++){
        // If parameter is valid
        if(targets->speed != -1.0f){

            // Adds the motor to the global bit mask
            motorBitMask |= (1 << i);
        }
    }

    // Log
    ESP_LOGD(TASK_NAME_CONTROL, "Motor Bit Mask: 0x%2X", motorBitMask);

    // Wait for all motors to be ready (Once they recieve targets from the kinematics task)
    xEventGroupWaitBits(rtosResources->motorReady, motorBitMask, pdTRUE, pdTRUE, portMAX_DELAY);
    
    // Enable Motors
    xEventGroupClearBits(rtosResources->motorIdle, motorBitMask);
    xEventGroupSetBits(rtosResources->motorEnabled, motorBitMask);

    // Wait until all motor's are idle before continuing
    xEventGroupWaitBits(rtosResources->motorIdle, motorBitMask, pdFALSE, pdTRUE, portMAX_DELAY);
}

void setAnglesCalc(RtosResources* rtosResources, void* args){

}
// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// ~~ sleep(ms) ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

// Puts the controller to sleep for a given time in ms
void sleepSM(RtosResources* rtosResources, void* args){

    // Cast and delete args
    int timeMS = *(int*)args;
    delete (int*)args;

    // Delay
    ESP_LOGD(TASK_NAME_CONTROL, "Delaying control task for %dms", timeMS);
    vTaskDelay(pdMS_TO_TICKS(timeMS));
}