// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// ~~ Libraries ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
#include "config.h"
#include "motorModule.h"
#include "kinematics_task.h"

// ESP/FreeRTOS
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

// Utils
#include <cmath>
#include <cstring>

// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// ~~ Constructor/Destructor ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

// Constructor
KinematicsTask::KinematicsTask(){

}

// Destructor
KinematicsTask::~KinematicsTask(){

}

// ********************************* PRIVATE **************************************

// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// ~~ Task Definition ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

void KinematicsTask::taskEntry(void* pvParameter){
    KinematicsTask* self = static_cast<KinematicsTask*>(pvParameter);
    self->kinematicsTask();
}

void KinematicsTask::kinematicsTask(){

    // Task Variables
    UserCommand cmd;
    bool cmdFound;

    // Intialize virtMotorAngle
    for(int i = 0; i < 6; i++){
        
        // Check to see if the motor is defined
        if(motors[i] != nullptr){

            // Wait for the motor to be idle
            xEventGroupWaitBits(rtosResources->motorIdle, (1 << i), pdFALSE, pdTRUE, portMAX_DELAY);

            // Measures it's start angle
            virtAngle[i] = motors[i]->updateAngle();
        }
    }

    ESP_LOGI(TASK_NAME_KINEMATICS, "Kinematics Task Initialized");

    while(true){

        // Wait until user command struct is recieved
        xQueueReceive(rtosResources->kinematicsCmd, &cmd, portMAX_DELAY);

        // Search through all robotTasks
        cmdFound = false;
        for(int i = 0; i < numRobotTasks; i++){
            // If the user command matches a robotTask command

            if(strcmp(robotTasks[i].name,cmd.name) == 0){
                // Then we run the state machine function, only if there is a valid function for it
                cmdFound = true;

                if(robotTasks[i].calculationFunc != nullptr){
                    robotTasks->calculationFunc(this, cmd.args);
                }
                else{
                    ESP_LOGD(TASK_NAME_KINEMATICS, "No kinematics calculation function found");
                }
                break;
            }
        }
        if(!cmdFound){
            ESP_LOGE(TASK_NAME_KINEMATICS, "Unknown command name");
        }

    }

}

// *********************************** PUBLIC **************************************

// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// ~~ Task Controls ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

// Starts the task
void KinematicsTask::start(){
    if(!isInitialized){
        ESP_LOGE(TASK_NAME_KINEMATICS, "Task has not been initialized yet, returning...");
        return;
    }
    else if(taskHandle != nullptr){
        ESP_LOGE(TASK_NAME_KINEMATICS, "Task is already running, returning...");
        return;
    }
    xTaskCreate(&KinematicsTask::taskEntry, TASK_NAME_KINEMATICS, TASK_STACK_DEPTH_KINEMATICS, this, TASK_PRIORITY_KINEMATICS, &taskHandle);
    ESP_LOGI(TASK_NAME_KINEMATICS, "Task started");
}

// Stops the task
void KinematicsTask::stop(){
    vTaskDelete(taskHandle);
    ESP_LOGI(TASK_NAME_KINEMATICS, "Task Stopped");
}

// Restarts the task, if it's running
void KinematicsTask::restart(){
    if(taskHandle != nullptr){
        stop();
    }
    start();
}

// RTOS Resources
RtosResources* KinematicsTask::getRtosResources(){
    return rtosResources;
}

// Initializes class parameters
void KinematicsTask::init(RtosResources* resources, MotorModule** motors){
    if(isInitialized){
        ESP_LOGE(TASK_NAME_KINEMATICS, "Task has already been initialized");
        return;
    }
    else{
        this->motors = motors;
        this->rtosResources = resources;
        isInitialized = true;
        ESP_LOGI(TASK_NAME_KINEMATICS, "Task has been succesfully initialized");
    }
}