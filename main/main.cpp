#include "config.h"
#include "robotic_definitions.h"
#include "as5600.h"
#include "pca9548a.h"
#include "stepper.h"
#include "motor_task.h"

#include "driver/ledc.h"

#include <chrono> // For time in ms
#include <thread> // For sleeping
#include <cstdint> // for uint16_t
#include <iostream>
#include <iomanip>
#include "esp_log.h"

EventGroupHandle_t motorEnable;
EventGroupHandle_t motorIdle;
QueueHandle_t j6sDesiredAngleQueue;
QueueHandle_t j6sParamsQueue;

extern "C" void app_main(){

    // Initialize RTOS Queues and Event Groups
    j6sDesiredAngleQueue = xQueueCreate(10, sizeof(float));
    j6sParamsQueue = xQueueCreate(10, sizeof(MotorParams));
    motorEnable = xEventGroupCreate();
    motorIdle = xEventGroupCreate();

    if (motorIdle == NULL) {
        ESP_LOGE("EventGroup", "Failed to create motorIdle event group!");
        return;
    }

    // PCA9548A
    Pca9548aParams pca9548aParams = {I2C_SCL_PIN, I2C_SDA_PIN, PCA9548A_SLAVE_ADDR, I2C_FREQ, I2C_NUM_0};
    PCA9548A pca9548a(pca9548aParams);

    // Motor 6
    AS5600 j6sAS5600(&pca9548a, J6S_PORT, CONF);
    StepperMotor j6sMotor(J6S_PIN_STEP, J6S_PIN_DIR);
    MotorParams j6sParams = {&j6sMotor, &j6sAS5600, STEPPER_SPEED, BIT5, j6sDesiredAngleQueue, j6sParamsQueue};
    xTaskCreate(motorTask, J6S_TASK_NAME, 2048, &j6sParams, 1, NULL);

    // Delay 5 seconds
    std::this_thread::sleep_for(std::chrono::milliseconds(5000));

    // Go to Angle 0
    float desiredAngle = 0.0f;
    xQueueSend(j6sDesiredAngleQueue, &desiredAngle, 0);
    xEventGroupSetBits(motorEnable, BIT5);

    // Wait a ms for idle flag to shut off
    std::this_thread::sleep_for(std::chrono::milliseconds(1));

    // Wait until motor is idle again
    xEventGroupWaitBits(motorIdle, BIT5, pdFALSE, pdFALSE, portMAX_DELAY);

    // Go to Angle 360
    desiredAngle = 360.0f;
    xQueueSend(j6sDesiredAngleQueue, &desiredAngle, 0);
    xEventGroupSetBits(motorEnable, BIT5);

    // Wait a ms for idle flag to shut off
    std::this_thread::sleep_for(std::chrono::milliseconds(1));

    // Wait until motor is idle again
    xEventGroupWaitBits(motorIdle, BIT5, pdFALSE, pdFALSE, portMAX_DELAY);

    // Go to Angle 180
    desiredAngle = 180.0f;
    xQueueSend(j6sDesiredAngleQueue, &desiredAngle, 0);
    xEventGroupSetBits(motorEnable, BIT5);

    // Wait a ms for idle flag to shut off
    std::this_thread::sleep_for(std::chrono::milliseconds(1));

    // Wait until motor is idle again
    xEventGroupWaitBits(motorIdle, BIT5, pdFALSE, pdFALSE, portMAX_DELAY);




    

    while(true){
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    }

}