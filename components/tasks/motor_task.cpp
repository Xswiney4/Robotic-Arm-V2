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
// ~~ Variables  ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// ~~ Task Definitions ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

// Responsible for driving a single motor
void motorTask(void *pvParameter){

    // Cast Motor Params
    MotorModule *motor = (MotorModule *)pvParameter;

    
    // Get motor number
    int motorNum = 0;
    for(; motorNum < 10; motorNum++){
        if(motor->bitMask & (1 << motorNum)){
            break;
        }
    }

    // Check if no motor number is found
    if(motorNum == 9){
        ESP_LOGE(pcTaskGetName(NULL), "Motor number not found, deleting task...");
        vTaskDelete(NULL);
    }
    else{
        ESP_LOGI(pcTaskGetName(NULL), "Motor %d Initialized", motorNum);
    }

    // Task Variables
    TargetParams target;
    
    // Task
    while(true){

        // Wait for targetParams
        xQueueReceive(motorTargetsQueue[motorNum], &target, portMAX_DELAY);

        // Wait for an enable signal from the control task
        xEventGroupWaitBits(motorEnable, motor->bitMask, pdFALSE, pdTRUE, portMAX_DELAY);

        // Move to target
        motor->setAngle(target);

        // Remove the enable signal
        xEventGroupClearBits(motorEnable, motor->bitMask);

    }
}

// Responsible for monitoring and validating the step count of all the motors
void stepMonitorTask(void *pvParameter){

    // Cast Motor Objects
    MotorModule *motors = (MotorModule *)pvParameter;
    ESP_LOGI(pcTaskGetName(NULL), "Step Monitor Task Initialized");

    EventBits_t uxBits;

    // Step Rate Setup
    TickType_t xFrequency = pdMS_TO_TICKS(1000.0f / STEP_MONITOR_CHECK_TIME);
    TickType_t xLastWakeTime = xTaskGetTickCount();
    
    while(true){

        // Wait for a motor to be enabled
        uxBits = xEventGroupWaitBits(motorEnable, ALL_MOTORS_BIT_MASK, pdFALSE, pdFALSE, portMAX_DELAY);

        // Check all motors
        for(int i = 0; i < 6; i++){
            if(uxBits & (1 << i)){
                // If the motor is enabled
                motors[i].validateCurrentStep();
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

