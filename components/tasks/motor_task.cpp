// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// ~~ Libraries ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//#include "robotic_definitions.h"
#include "motor_task.h"

// ESP/FreeRTOS
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

// Utils
#include <cmath>

// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// ~~ Variables  ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

// Tolerance in degrees within which the motor angle is considered close enough to the desired angle to stop
const float tolerance = 1.5f;

// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// ~~ Task Definitions ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

// Responsible for controlling the motor
void motorTask(void *pvParameter){

    // Cast Motor Params
    MotorParams *params = (MotorParams *)pvParameter;

    // Variables
    float desiredAngle;

    ESP_LOGI(pcTaskGetName(NULL), "Motor Initialized");
    
    while(true){

        // 'Reset'
        desiredAngle = -1; // Reset desiredAngle
        xEventGroupSetBits(motorIdle, params->eventGroupBit); // Sets the 'motorIdle' flag

        // Given the speed, we need to take steps towards the target angle at intervals until we reach the target

        // Wait until we get a desired angle
        xQueueReceive(params->desiredAngleQueueHandle, &desiredAngle, portMAX_DELAY);

        setMotorAngle(params, desiredAngle);

    }
}

// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// ~~ Motor Functions ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

void setMotorAngle(MotorParams* params, float angle){

    // Variables
    float currentAngle;
    float lastAngle;
    float stepTriggerTime = 1800 / params->speed;
    TickType_t xFrequency = pdMS_TO_TICKS(stepTriggerTime);

    currentAngle = params->as5600->getAngle();
    ESP_LOGI(pcTaskGetName(NULL), "Start Angle: %f",currentAngle);

    // Set the direction
    if(currentAngle - tolerance > angle){
        params->stepper->setDir(true);
    }
    else if(currentAngle + tolerance < angle){
        params->stepper->setDir(false);
    }
    else{
        // If the current angle is already at the desired, we reset
        ESP_LOGI(pcTaskGetName(NULL), "Motor already at desired angle");
        return;
    }

    // Mark motor as ready
    xEventGroupSetBits(motorReady, params->eventGroupBit);

    // Waits until it's enable flag is set, and clears the flag
    xEventGroupWaitBits(motorEnable, params->eventGroupBit, pdTRUE, pdFALSE, portMAX_DELAY);

    // Clears the 'motorIdle' flag
    xEventGroupClearBits(motorIdle, params->eventGroupBit);

    // RTOS Tick Setup
    TickType_t xLastWakeTime = xTaskGetTickCount();

    // Take steps until we reach the target
    while(currentAngle - tolerance > angle || currentAngle + tolerance < angle){

        vTaskDelayUntil(&xLastWakeTime, xFrequency); // Block until delay

        params->stepper->step(); // Take a step
        lastAngle = currentAngle;
        currentAngle = params->as5600->getAngle(); // Updates currentAngle

        // Checks to see if the AS5600 jumped
        if(std::abs(lastAngle - currentAngle) > 100.0f){
            if(lastAngle > 180){ // We're at the 360 threshold
                currentAngle = 360.0f;
            }
            else{ // We're at the 0 threshold
                currentAngle = 0.0f;
            }
        }
        ESP_LOGI(pcTaskGetName(NULL), "Current Angle: %f",currentAngle);
    }

    ESP_LOGI(pcTaskGetName(NULL), "Motor has reached desired angle");

}

// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// ~~ Other Utils ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
