// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// ~~ Libraries ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
#include "robotic_definitions.h"
#include "config.h"
#include "motorModule.h"
#include "kinematics_task.h"

// ESP/FreeRTOS
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

// Utils
#include <cmath>

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
    self->motors = (MotorModule** )pvParameter;
    self->kinematicsTask();
}

void KinematicsTask::kinematicsTask(){

    // Task Variables
    UserCommand cmd;
    float virtMotorAngle[6]; // This is the location each motor should be at before it performs a calculation

    // Intialize virtMotorAngle
    for(int i = 0; i < 6; i++){
        
        // Check to see if the motor is defined
        if(motors[i] != nullptr){

            // Wait for the motor to be idle
            xEventGroupWaitBits(motorIdle, (1 << i), pdFALSE, pdTRUE, portMAX_DELAY);

            // Measures it's start angle
            virtMotorAngle[i] = motors[i]->updateAngle();
        }
    }

    ESP_LOGI(TASK_NAME_KINEMATICS, "Kinematics Task Initialized");

    while(true){

        // Wait until user command struct is recieved
        xQueueReceive(kinematicsCmd, &cmd, portMAX_DELAY);

        // Now we decode the command struct
        switch (cmd.commandNum){
            // Invalid Command
            case -1:
                ESP_LOGE(TASK_NAME_KINEMATICS, "Invalid command");
                break;
                
            case 0:
                ESP_LOGI(TASK_NAME_KINEMATICS, "Successfully Started setEndKinCalc()");
                setEndKinCalc(&cmd, motors, virtMotorAngle);
                break;

            case 10:
                ESP_LOGI(TASK_NAME_KINEMATICS, "Successfully Started setMotorAnglesKinCalc()");
                setMotorAnglesKinCalc(&cmd, motors, virtMotorAngle);
                break;

            default:
                ESP_LOGD(TASK_NAME_KINEMATICS, "No processing required, throwing out command");
                break;
        }

    }

}

// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// ~~ User Commands ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

// Converts a position and orientation into joint angles, and then sends them to the control task
void KinematicsTask::setEndKinCalc(UserCommand* cmd, MotorModule** motors, float virtMotorAngle[]){

    // Variable Declaration
    // float x = cmd->params[0];
    // float y = cmd->params[1];
    // float z = cmd->params[2];
    // float pitch = cmd->params[3];
    // float yaw = cmd->params[4];
    // float roll = cmd->params[5];


}

// Converts a position and orientation into joint angles, and then sends them to the control task
void KinematicsTask::setMotorAnglesKinCalc(UserCommand* cmd, MotorModule** motors, float virtMotorAngle[]){

    // Variable Declaration
    // float m1Angle =    cmd->params[0];
    // float m2Angle =    cmd->params[1];
    // float m3Angle =    cmd->params[2];
    // float m4Angle =    cmd->params[3];
    // float m5Angle =    cmd->params[4];
    // float m6Angle =    cmd->params[5];

    // Loop going through each parameter input
    for(int i = 0; i < 6; i++){
        // If parameter is valid
        if(cmd->params[i] != -1){

            TargetParams target;

            // Difference the input motor needs to move in order for the output angle to reach the target
            float inputDiff = cmd->params[i] - virtMotorAngle[i];

            // Calculate the number of steps needed
            target.numSteps = (int)roundf(inputDiff / motors[i]->degreesPerStep);
            
            
            // Calculates the step time in ms
            float stepPeriodMs = 1800.0f / (STEPPER_SPEED * motors[i]->degreesPerStep);

            target.xFrequency = pdMS_TO_TICKS(stepPeriodMs);
            target.targetAngle = cmd->params[i];


            // Send target to motor
            xQueueSend(motorTargetsQueue[i], &target, portMAX_DELAY);

            // Set the virtMotorAngle and
            virtMotorAngle[i] = cmd->params[i];

        }
    }

}

// *********************************** PUBLIC **************************************

// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// ~~ Task Controls ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

// Starts the task
void KinematicsTask::start(){
    if(taskHandle != nullptr){
        ESP_LOGE(TASK_NAME_KINEMATICS, "Task is already running, returning...");
        return;
    }
    xTaskCreate(&KinematicsTask::taskEntry, TASK_NAME_KINEMATICS, TASK_STACK_DEPTH_KINEMATICS, NULL, TASK_PRIORITY_KINEMATICS, &taskHandle);
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
