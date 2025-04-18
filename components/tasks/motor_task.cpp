// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// ~~ Libraries ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//#include "robotic_definitions.h"
#include "config.h"
#include "motorModule.h"
#include "motor_task.h"

// ESP/FreeRTOS
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

// Utils
#include <cmath>

// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// ~~ Task Definitions ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

// Responsible for driving a single motor
void motorTask(void *pvParameter){

    // Cast Motor Params
    MotorModule* motor = (MotorModule* )pvParameter;

    ESP_LOGI(pcTaskGetName(NULL), "Motor Initialized");

    // Task Variables
    TargetParams target;
    
    // Task
    while(true){

 
        // Set motor to idle
        xEventGroupSetBits(motorIdle, motor->bitMask);
        ESP_LOGD(pcTaskGetName(NULL), "Motor Idle");

        // Wait for targetParams
        xQueueReceive(motorTargetsQueue[motor->motorNum], &target, portMAX_DELAY);

        // Mark as ready
        xEventGroupSetBits(motorReady, motor->bitMask);
        ESP_LOGD(pcTaskGetName(NULL), "Motor Ready");

        // Wait for an enable signal from the control task
        xEventGroupWaitBits(motorEnabled, motor->bitMask, pdFALSE, pdTRUE, portMAX_DELAY);
        ESP_LOGD(pcTaskGetName(NULL), "Motor Enabled");

        // Move to target
        motor->setAngle(target);

        // Remove the enable signal
        xEventGroupClearBits(motorEnabled, motor->bitMask);

    }
}

// Responsible for monitoring and validating the step count of all the motors
void stepMonitorTask(void *pvParameter){

    // Cast Motor Objects
    MotorModule** motors = (MotorModule** )pvParameter;
    ESP_LOGI(pcTaskGetName(NULL), "Step Monitor Task Initialized");

    EventBits_t uxBits;

    // Step Rate Setup
    TickType_t xFrequency = pdMS_TO_TICKS(1000.0f / STEP_MONITOR_CHECK_TIME);
    TickType_t xLastWakeTime = xTaskGetTickCount();
    
    while(true){

        // Wait for a motor to be enabled
        uxBits = xEventGroupWaitBits(motorEnabled, ALL_MOTORS_BIT_MASK, pdFALSE, pdFALSE, portMAX_DELAY);

        // Check all motors
        for(int i = 0; i < 6; i++){
            if(uxBits & (1 << i)){
                // If the motor is enabled, validate
                motors[i]->validateCurrentStep();

                // Delay the next validation until the next FreeRTOS tick
                // (The goal is have one validation event occur per tick)
                vTaskDelay(1);
            }
        }
        
        // After checking all motors, delay for a time
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// ~~ Motor Functions ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// ~~ Motor Helper Functions ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

