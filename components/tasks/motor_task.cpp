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
// ~~ Initializations ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


#include "config.h"
// Creates all 6 motor tasks given the configurations
void initMotors(){
    
    // PCA9548A
    Pca9548aParams pca9548aParams = {I2C_SCL_PIN, I2C_SDA_PIN, PCA9548A_SLAVE_ADDR, I2C_FREQ, I2C_NUM_0};
    PCA9548A pca9548a(pca9548aParams);
    
    // Motor 1
    AS5600 j1sAS5600(&pca9548a, J1S_PORT, CONF);
    StepperMotor j1sMotor(J1S_PIN_STEP, J1S_PIN_DIR);
    MotorParams j1sParams = {&j1sMotor, &j1sAS5600, STEPPER_SPEED, BIT0, j1sDesiredAngleQueue, j1sParamsQueue};
    xTaskCreate(motorTask, J1S_TASK_NAME, 2048, &j1sParams, 1, NULL);

    // Motor 2
    AS5600 j2sAS5600(&pca9548a, J2S_PORT, CONF);
    StepperMotor j2sMotor(J2S_PIN_STEP, J2S_PIN_DIR);
    MotorParams j2sParams = {&j2sMotor, &j2sAS5600, STEPPER_SPEED, BIT1, j2sDesiredAngleQueue, j2sParamsQueue};
    xTaskCreate(motorTask, J2S_TASK_NAME, 2048, &j2sParams, 1, NULL);

    // Motor 3
    AS5600 j3sAS5600(&pca9548a, J3S_PORT, CONF);
    StepperMotor j3sMotor(J3S_PIN_STEP, J3S_PIN_DIR);
    MotorParams j3sParams = {&j3sMotor, &j3sAS5600, STEPPER_SPEED, BIT2, j3sDesiredAngleQueue, j3sParamsQueue};
    xTaskCreate(motorTask, J3S_TASK_NAME, 2048, &j3sParams, 1, NULL);

    // Motor 4
    AS5600 j4sAS5600(&pca9548a, J4S_PORT, CONF);
    StepperMotor j4sMotor(J4S_PIN_STEP, J4S_PIN_DIR);
    MotorParams j4sParams = {&j4sMotor, &j4sAS5600, STEPPER_SPEED, BIT3, j4sDesiredAngleQueue, j4sParamsQueue};
    xTaskCreate(motorTask, J4S_TASK_NAME, 2048, &j4sParams, 1, NULL);

    // Motor 5
    AS5600 j5sAS5600(&pca9548a, J5S_PORT, CONF);
    StepperMotor j5sMotor(J5S_PIN_STEP, J5S_PIN_DIR);
    MotorParams j5sParams = {&j5sMotor, &j5sAS5600, STEPPER_SPEED, BIT4, j5sDesiredAngleQueue, j5sParamsQueue};
    xTaskCreate(motorTask, J5S_TASK_NAME, 2048, &j5sParams, 1, NULL);

    // Motor 6
    AS5600 j6sAS5600(&pca9548a, J6S_PORT, CONF);
    StepperMotor j6sMotor(J6S_PIN_STEP, J6S_PIN_DIR);
    MotorParams j6sParams = {&j6sMotor, &j6sAS5600, STEPPER_SPEED, BIT5, j6sDesiredAngleQueue, j6sParamsQueue};
    xTaskCreate(motorTask, J6S_TASK_NAME, 2048, &j6sParams, 1, NULL);

}


// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// ~~ Task Definitions ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

// Responsible for controlling the motor
void motorTask(void *pvParameter){

    // Cast Motor Params
    MotorParams *params = (MotorParams *)pvParameter;

    // Variables
    float desiredAngle;
    float currentAngle;

    float tolerance = 1.0f;

    
    while(true){

        // 'Reset'
        desiredAngle = -1; // Reset desiredAngle
        xEventGroupSetBits(motorIdle, params->eventGroupBit); // Sets the 'motorIdle' flag

        // Waits until it's enable flag is set, and clears the flag
        xEventGroupWaitBits(motorEnable, params->eventGroupBit, pdTRUE, pdFALSE, portMAX_DELAY);

        // Clears the 'motorIdle' flag
        xEventGroupClearBits(motorIdle, params->eventGroupBit);

        // Given the speed, we need to take steps towards the target angle at intervals until we reach the target

        // First we get the desired angle
        xQueueReceive(params->desiredAngleQueueHandle, &desiredAngle, 0);

        // Error Check
        if(desiredAngle == -1){
            ESP_LOGE(pcTaskGetName(NULL), "Motor enabled without sending a desired angle, restarting task...");
            break;
        }

        currentAngle = params->as5600->getAngle();
        ESP_LOGI(pcTaskGetName(NULL), "Start Angle: %f",currentAngle);

        // Set the direction
        if(currentAngle - tolerance > desiredAngle){
            params->stepper->setDir(true);
        }
        else if(currentAngle + tolerance < desiredAngle){
            params->stepper->setDir(false);
        }
        else{
            // If the current angle is already at the desired, we reset
            ESP_LOGI(pcTaskGetName(NULL), "Motor already at desired angle");
            break;
        }

        // Calculate time between steps
        float difference = std::abs(currentAngle - desiredAngle); // Difference (deg)

        float numSteps = std::round(difference / 1.8f); // Total steps need to get close to the desired (steps)

        float timeTotal = difference / params->speed; // Total time needed to match the set speed (s)

        float stepTriggerTime = 1000 * timeTotal / numSteps; // Total time to wait after each step (ms)

        // RTOS Tick Setup
        TickType_t xLastWakeTime = xTaskGetTickCount();
        TickType_t xFrequency = stepTriggerTime / portTICK_PERIOD_MS;

        // Take steps until we reach the target
        while(currentAngle - tolerance > desiredAngle || currentAngle + tolerance < desiredAngle){

            vTaskDelayUntil(&xLastWakeTime, xFrequency); // Block until delay

            params->stepper->step(); // Take a step
            currentAngle = params->as5600->getAngle(); // Updates currentAngle
            ESP_LOGI(pcTaskGetName(NULL), "Current Angle: %f",currentAngle);
        }

        ESP_LOGI(pcTaskGetName(NULL), "Motor has reached desired angle");

        

    }
}

// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// ~~ Other Utils ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
