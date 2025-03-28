// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// ~~ Libraries ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//#include "robotic_definitions.h"
#include "config.h"
#include "motor_task.h"

// ESP/FreeRTOS
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

// Utils
#include <cmath>

// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// ~~ Variables  ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// ~~ Task Definitions ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

// Responsible for controlling the motor
void motorTask(void *pvParameter){

    // Cast Motor Params
    MotorParams *params = (MotorParams *)pvParameter;
    
    // Get starting angle
    updateAngle(params);


    ESP_LOGI(pcTaskGetName(NULL), "Motor Initialized");
    
    while(true){

        // 'Reset'
        params->targetAngle     = -1;
        params->targetSpeed     = -1;
        params->xFrequency      = (TickType_t) -1;
        xEventGroupSetBits(motorIdle, params->eventGroupBit); // Sets the 'motorIdle' flag

        // Given the speed, we need to take steps towards the target angle at intervals until we reach the target

        // Waits until it's enable flag is set, and clears the flag
        xEventGroupWaitBits(motorEnable, params->eventGroupBit, pdTRUE, pdFALSE, portMAX_DELAY);

        setMotorAngle(params);

    }
}

// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// ~~ Motor Functions ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

void setMotorAngle(MotorParams* params){

    // Speed calculation
    if(params->xFrequency != (TickType_t) -1){
        // If xFrequency is defined, we use that. (No further calculation required)
    }
    else if(params->targetSpeed != -1){
        // If speed is defined, we use that
        params->xFrequency = pdMS_TO_TICKS(roundf(1800.0f / params->targetSpeed));
    }
    else{
        // Otherwise, we go with the global STEPPER_SPEED variable
        params->xFrequency = pdMS_TO_TICKS(roundf(1800.0f / STEPPER_SPEED));
    }

    // Set direction
    setDir(params);

    ESP_LOGD(pcTaskGetName(NULL), "Start Angle: %f",params->currentAngle);

    // RTOS Tick Setup
    TickType_t xLastWakeTime = xTaskGetTickCount();

    // Take steps until we reach the target
    while(true){
        
        vTaskDelayUntil(&xLastWakeTime, params->xFrequency); // Block until delay

        updateAngle(params);

        if(fabs(params->currentAngle - params->targetAngle) <= MOTOR_ANGLE_TOLERANCE){
            break;
        }

        params->stepper->step(); // Take a step

    }

    ESP_LOGD(pcTaskGetName(NULL), "Motor has reached desired angle");

}

// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// ~~ Motor Helper Functions ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

// Set the direction based on the current angle and target angle
void setDir(MotorParams* params){
    if(params->currentAngle - MOTOR_ANGLE_TOLERANCE > params->targetAngle){
        ESP_LOGD(pcTaskGetName(NULL), "Set motor direction 'true'");
        params->stepper->setDir(true);
    }
    else if(params->currentAngle + MOTOR_ANGLE_TOLERANCE < params->targetAngle){
        ESP_LOGD(pcTaskGetName(NULL), "Set motor direction 'false'");
        params->stepper->setDir(false);
    }
    else{
        // If the current angle is already at the desired, we reset
        ESP_LOGD(pcTaskGetName(NULL), "Motor already at desired angle: %f", params->targetAngle);
        return;
    }
}

// Measures and updates the currentAngle variable
void updateAngle(MotorParams* params){
    params->currentAngle = params->as5600->getAngle();
    ESP_LOGD(pcTaskGetName(NULL), "Angle updated: %f",params->currentAngle);
}