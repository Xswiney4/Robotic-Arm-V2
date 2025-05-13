// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// ~~ Libraries ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
#include "config.h"
#include "motor_monitor_task.h"

// ESP/FreeRTOS
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

// Utils
#include <cmath>

// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// ~~ Constructor/Destructor ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

// Constructor
MotorMonitorTask::MotorMonitorTask(){

}

// Destructor
MotorMonitorTask::~MotorMonitorTask(){

}

// ********************************* PRIVATE **************************************

// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// ~~ Task Definition ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

void MotorMonitorTask::taskEntry(void* pvParameter){
    MotorMonitorTask* self = static_cast<MotorMonitorTask*>(pvParameter);
    self->stepMonitorTask();
}

// Responsible for monitoring and validating the step count of all the motors
void MotorMonitorTask::stepMonitorTask(){

    ESP_LOGI(TASK_NAME_STEP_MONITOR, "Step Monitor Task Initialized");

    EventBits_t uxBits;

    // Step Rate Setup
    TickType_t xFrequency = pdMS_TO_TICKS(1000.0f / STEP_MONITOR_CHECK_TIME);
    TickType_t xLastWakeTime = xTaskGetTickCount();
    
    while(true){

        // Wait for a motor to be enabled
        uxBits = xEventGroupWaitBits(rtosResources->motorEnabled, ALL_MOTORS_BIT_MASK, pdFALSE, pdFALSE, portMAX_DELAY);

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

// *********************************** PUBLIC **************************************

// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// ~~ Task Controls ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

// Starts the task
void MotorMonitorTask::start(){
    if(!isInitialized){
        ESP_LOGE(TASK_NAME_STEP_MONITOR, "Task has not been initialized yet, returning...");
        return;
    }
    else if(taskHandle != nullptr){
        ESP_LOGE(TASK_NAME_STEP_MONITOR, "Task is already running, returning...");
        return;
    }
    xTaskCreate(&MotorMonitorTask::taskEntry, TASK_NAME_STEP_MONITOR, TASK_STACK_DEPTH_STEP_MONITOR, this, TASK_PRIORITY_STEP_MONITOR, &taskHandle);
    ESP_LOGI(TASK_NAME_STEP_MONITOR, "Task started");
}

// Stops the task
void MotorMonitorTask::stop(){
    vTaskDelete(taskHandle);
    ESP_LOGI(TASK_NAME_STEP_MONITOR, "Task Stopped");
}

// Restarts the task, if it's running
void MotorMonitorTask::restart(){
    if(taskHandle != nullptr){
        stop();
    }
    start();
}

// RTOS Resources
RtosResources* MotorMonitorTask::getRtosResources(){
    return rtosResources;
}

// Initializes class parameters
void MotorMonitorTask::init(RtosResources* resources, MotorModule** motors){
    if(isInitialized){
        ESP_LOGE(TASK_NAME_STEP_MONITOR, "Task has already been initialized");
        return;
    }
    else{
        this->motors = motors;
        this->rtosResources = resources;
        isInitialized = true;
        ESP_LOGI(TASK_NAME_STEP_MONITOR, "Task has been succesfully initialized");
    }
}