// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// ~~ Libraries ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
#include "control_task.h"
#include "motorModule.h"
#include "config.h"

// ESP/FreeRTOS
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"

// Utils
#include <cstring>

// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// ~~ Constructor/Destructor ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

// Constructor
ControlTask::ControlTask(){

}

// Destructor
ControlTask::~ControlTask(){

}

// ********************************* PRIVATE **************************************

// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// ~~ Task Definitions ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

void ControlTask::taskEntry(void* pvParameters){
    ControlTask* self = static_cast<ControlTask*>(pvParameters);
    self->controlTask();
}

void ControlTask::controlTask(){

    // Task Variables
    UserCommand cmd;
    bool cmdFound;

    ESP_LOGI(TASK_NAME_CONTROL, "Control Task Initialized");

    while(true){

        // Wait until user command struct is recieved
        xQueueReceive(rtosResources->controlCmd, &cmd, portMAX_DELAY);

        // Search through all robotTasks
        cmdFound = false;
        for(int i = 0; i < numRobotTasks; i++){
            // If the user command matches a robotTask command
            if(strcmp(robotTasks[i].name,cmd.name) == 0){
                // Then we run the state machine function
                cmdFound = true;
                robotTasks->stateMachineFunc;
                break;
            }
        }
        if(!cmdFound){
            ESP_LOGE(TASK_NAME_CONTROL, "Unknown command name");
        }

    }
}

// *********************************** PUBLIC **************************************

// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// ~~ Task Controls ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

// Starts the task
void ControlTask::start(){
    if(!isInitialized){
        ESP_LOGE(TASK_NAME_CONTROL, "Task has not been initialized yet, returning...");
        return;
    }
    else if(taskHandle != nullptr){
        ESP_LOGE(TASK_NAME_CONTROL, "Task is already running, returning...");
        return;
    }
    xTaskCreate(&ControlTask::taskEntry, TASK_NAME_CONTROL, TASK_STACK_DEPTH_CONTROL, this, TASK_PRIORITY_CONTROL, &taskHandle);
    ESP_LOGI(TASK_NAME_CONTROL, "Task started");
}

// Stops the task
void ControlTask::stop(){
    vTaskDelete(taskHandle);
    ESP_LOGI(TASK_NAME_CONTROL, "Task Stopped");
}

// Restarts the task, if it's running
void ControlTask::restart(){
    if(taskHandle != nullptr){
        stop();
    }
    start();
}

// Initializes class parameters
void ControlTask::init(RtosResources* resources){
    if(isInitialized){
        ESP_LOGE(TASK_NAME_CONTROL, "Task has already been initialized");
        return;
    }
    else{
        isInitialized = true;
        rtosResources = resources;
        ESP_LOGI(TASK_NAME_CONTROL, "Task has been succesfully initialized");
    }
}