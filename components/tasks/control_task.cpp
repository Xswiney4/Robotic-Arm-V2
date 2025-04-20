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

    ESP_LOGI(TASK_NAME_CONTROL, "Control Task Initialized");

    while(true){

        // Wait until user command struct is recieved
        xQueueReceive(controlCmd, &cmd, portMAX_DELAY);

        // Now we decode the command struct
        switch (cmd.commandNum){
            // Invalid Command
            case -1:
                ESP_LOGE(TASK_NAME_CONTROL, "Invalid command");
                break;
                
            case 0:
                ESP_LOGD(TASK_NAME_CONTROL, "Successfully Started setEnd()");
                setEnd(&cmd);
                break;
            
            case 1:
                ESP_LOGD(TASK_NAME_CONTROL, "Successfully Started setEndSpeed()");
                setEndSpeed(&cmd);
                break;
            
            case 10:
                ESP_LOGD(TASK_NAME_CONTROL, "Successfully Started setMotorAngles()");
                setMotorAngles(&cmd);
                break;

            case 11:
                ESP_LOGD(TASK_NAME_CONTROL, "Successfully Started setMotorSpeed()");
                setMotorSpeed(&cmd);
                break;

            case 20:
                ESP_LOGD(TASK_NAME_CONTROL, "Successfully Started sleep()");
                sleep(&cmd);
                break;

            default:
                ESP_LOGE(TASK_NAME_CONTROL, "Unknown cmd.commandNum");
                break;
        }

    }
}

// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// ~~ User Commands ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

/*
Sets the endpoint given a position (x, y, z) and orientation (pitch, yaw, roll) of the end effector
The control structure is as follows:
- UserCommand is sent over to Kinematics Solver to be solved
- Wait until all motors are ready AND Kinematic Solver flags 'solved'
- Enable all motors
- Break
*/
void ControlTask::setEnd(UserCommand* cmd){
    // double x =      cmd->params[0];
    // double y =      cmd->params[1];
    // double z =      cmd->params[2];
    // double pitch =  cmd->params[3];
    // double yaw =    cmd->params[4];
    // double roll =   cmd->params[5];
}

/*
Sets the endpoint speed in m/s
The structure is as follows:
- 
*/
void ControlTask::setEndSpeed(UserCommand* cmd){
    // double endSpeed = cmd->params[0];
}

/*
Sets the angle of a given motor in degrees, it uses STEPPER_SPEED as the speed for the motors
The structure is as follows:
- Angle is sent over to motor
- Wait until given motor is ready
- Enable given motor
- Break
*/
void ControlTask::setMotorAngles(UserCommand* cmd){
    // float m1Angle =    cmd->params[0];
    // float m2Angle =    cmd->params[1];
    // float m3Angle =    cmd->params[2];
    // float m4Angle =    cmd->params[3];
    // float m5Angle =    cmd->params[4];
    // float m6Angle =    cmd->params[5];
    
    uint8_t motorBitMask = 0x00;

    // Loop going through each parameter input
    for(int i = 0; i < 6; i++){
        // If parameter is valid
        if(cmd->params[i] != -1.0f){

            // Adds the motor to the global bit mask
            motorBitMask |= (1 << i);
        }
    }

    // Log
    ESP_LOGD(TASK_NAME_CONTROL, "Motor Bit Mask: 0x%2X", motorBitMask);

    // Wait for all motors to be ready (Once they recieve targets from the kinematics task)
    xEventGroupWaitBits(motorReady, motorBitMask, pdTRUE, pdTRUE, portMAX_DELAY);
    
    // Enable Motors
    xEventGroupClearBits(motorIdle, motorBitMask);
    xEventGroupSetBits(motorEnabled, motorBitMask);

    // Wait until all motor's are idle before continuing
    xEventGroupWaitBits(motorIdle, motorBitMask, pdFALSE, pdTRUE, portMAX_DELAY);

}

/*
Sets the given motor speed in m/s
The structure is as follows:
- UserCommand is sent over to Kinematics Solver to be solved
- Wait until given motor is ready AND Kinematic Solver flags 'solved'
- Break
*/
void ControlTask::setMotorSpeed(UserCommand* cmd){
    // int motor =     (int)cmd->params[0];
    // double speed =       cmd->params[1];
}

// Puts the controller to sleep for a given time in ms
void ControlTask::sleep(UserCommand* cmd){
    ESP_LOGD(TASK_NAME_CONTROL, "Delaying control task for %dms", (int)cmd->params[0]);
    vTaskDelay(pdMS_TO_TICKS((int)cmd->params[0]));
}

// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// ~~ Other Utils ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


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
    xTaskCreate(&ControlTask::taskEntry, TASK_NAME_CONTROL, TASK_STACK_DEPTH_CONTROL, NULL, TASK_PRIORITY_CONTROL, &taskHandle);
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
void ControlTask::init(){
    if(isInitialized){
        ESP_LOGE(TASK_NAME_CONTROL, "Task has already been initialized");
        return;
    }
    else{
        isInitialized = true;
        ESP_LOGI(TASK_NAME_CONTROL, "Task has been succesfully initizlized");
    }
}