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
// ~~ Constructor/Destructor ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

// Constructor
MotorTask::MotorTask(){

}

// Destructor
MotorTask::~MotorTask(){

}

// ********************************* PRIVATE **************************************

// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// ~~ Task Definitions ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

void MotorTask::taskEntry(void* pvParameter){
    MotorTask* self = static_cast<MotorTask*>(pvParameter);
    self->motorTask();
}

// Responsible for driving a single motor
void MotorTask::motorTask(){

    ESP_LOGI(pcTaskGetName(NULL), "Motor Initialized");

    // Task Variables
    TargetParams target;
    
    // Task
    while(true){

 
        // Set motor to idle
        xEventGroupSetBits(rtosResources->motorIdle, motor->bitMask);
        ESP_LOGD(pcTaskGetName(NULL), "Motor Idle");

        // Wait for targetParams
        xQueueReceive(rtosResources->motorTargetsQueue[motor->motorNum], &target, portMAX_DELAY);

        // Mark as ready
        xEventGroupSetBits(rtosResources->motorReady, motor->bitMask);
        ESP_LOGD(pcTaskGetName(NULL), "Motor Ready");

        // Wait for an enable signal from the control task
        xEventGroupWaitBits(rtosResources->motorEnabled, motor->bitMask, pdFALSE, pdTRUE, portMAX_DELAY);
        ESP_LOGD(pcTaskGetName(NULL), "Motor Enabled");

        // Move to target
        motor->setAngle(target);

        // Remove the enable signal
        xEventGroupClearBits(rtosResources->motorEnabled, motor->bitMask);

    }
}

// *********************************** PUBLIC **************************************

// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// ~~ Task Controls ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

// Starts the task
void MotorTask::start(){
    if(!isInitialized){
        ESP_LOGE(taskName, "Task has not been initialized yet, returning...");
        return;
    }
    else if(taskHandle != nullptr){
        ESP_LOGE(taskName, "Task is already running, returning...");
        return;
    }
    xTaskCreate(&MotorTask::taskEntry, taskName, TASK_STACK_DEPTH_MOTOR, this, TASK_PRIORITY_MOTOR, &taskHandle);
    ESP_LOGI(taskName, "Task started");
}

// Stops the task
void MotorTask::stop(){
    vTaskDelete(taskHandle);
    ESP_LOGI(taskName, "Task Stopped");
}

// Restarts the task, if it's running
void MotorTask::restart(){
    if(taskHandle != nullptr){
        stop();
    }
    start();
}

// RTOS Resources
RtosResources* MotorTask::getRtosResources(){
    return rtosResources;
}

// Initializes class parameters
void MotorTask::init(RtosResources* resources, const char* taskName, MotorModule* motor){
    if(isInitialized){
        ESP_LOGE(taskName, "Task has already been initialized");
        return;
    }
    else{
        this->taskName = taskName;
        this->motor = motor;
        this->rtosResources = resources;
        isInitialized = true;
        ESP_LOGI(taskName, "Task has been succesfully initialized");
    }
}