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


    ESP_LOGI(pcTaskGetName(NULL), "Motor Initialized");
    
    while(true){

        // 'Reset'
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

    // Variables
    float stepTriggerTime = 1800 / params->targetSpeed;
    TickType_t xFrequency = pdMS_TO_TICKS(stepTriggerTime);

    // If the current angle is undefined, we measure it
    if(params->currentAngle == -1){
        params->currentAngle = params->as5600->getAngle();
    }

    ESP_LOGD(pcTaskGetName(NULL), "Start Angle: %f",params->currentAngle);

    // Set the direction
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

        // Mark motor as ready to allow control task to pass
        xEventGroupSetBits(motorReady, params->eventGroupBit);

        ESP_LOGD(pcTaskGetName(NULL), "Motor already at desired angle: %f", params->targetAngle);
        return;
    }

    // RTOS Tick Setup
    TickType_t xLastWakeTime = xTaskGetTickCount();

    // Take steps until we reach the target
    while(fabs(params->currentAngle - params->targetAngle) >= MOTOR_ANGLE_TOLERANCE){
        
        ESP_LOGD(pcTaskGetName(NULL), "Entered loop");

        vTaskDelayUntil(&xLastWakeTime, xFrequency); // Block until delay

        params->stepper->step(); // Take a step

        params->currentAngle = params->as5600->getAngle(); // Updates currentAngle

        // // Checks to see if the AS5600 jumped
        // if(fabs(params->currentAngle - params->currentAngle) > 100.0f){
        //     if(params->currentAngle > 180){ // We're at the 360 threshold
        //         params->currentAngle = 360.0f;
        //     }
        //     else{ // We're at the 0 threshold
        //         params->currentAngle = 0.0f;
        //     }
        // }
        ESP_LOGD(pcTaskGetName(NULL), "Current Angle: %f",params->currentAngle);
    }

    ESP_LOGD(pcTaskGetName(NULL), "Motor has reached desired angle");

}

// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// ~~ Other Utils ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
