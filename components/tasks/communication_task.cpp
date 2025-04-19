// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// ~~ Libraries ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
#include "robotic_definitions.h"
#include "communication_task.h"
#include "config.h"

// ESP/FreeRTOS
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

// Utils
#include <cstring>
#include <cstdlib>

// Bluetooth

// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// ~~ Constructor/Destructor ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

// Constructor
CommunicationTask::CommunicationTask(){

}

// Destructor
CommunicationTask::~CommunicationTask(){

}

// ********************************* PRIVATE **************************************

// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// ~~ Initializations ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

void CommunicationTask::initBluetooth(){

}

void CommunicationTask::initUART(){

}

// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// ~~ Handlers ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

/*
This function decodes a character array and returns a userCommand structure.
The structure has the following format:
struct userCommand{
    int command;
    double param0;
    double param1;
    double param2;
    double param3;
    double param4;
    double param5;
};

The commands and their given parameters are specified here:
https://docs.google.com/spreadsheets/d/1nJw_hCjyDbuuJXKSwHuH-XOVeJgRIX9mN4UZ95cLr8M/edit?gid=162912065#gid=162912065
*/

UserCommand CommunicationTask::userCmdParser(char *buffer){
    
    // Buffers
    UserCommand cmd;
    char cmdBuff[64];
    char paramBuff[32];

    // Character index
    int i;
    bool found = false;

    // Search for the first '('
    for(i = 0; i < 64; i++){
        if(buffer[i] == '('){
            cmdBuff[i] = '\0'; // Close up the character array
            i++;               // Move onto the next part in the buffer
            found = true;
            break;
        }
        cmdBuff[i] = buffer[i];
    }
    
    // Error Check
    if(!found){
        ESP_LOGE(TASK_NAME_COMMUNICATION, "Invalid User Command - Could not find '(' within string");
        return {-1};
    }
    // At this point, cmdBuff is our command
    
    // Now we search through the commands to see if it's valid
    int j;
    found = false;
    for(j = 0; j < numCommands; j++){
        if(strcmp(cmdBuff,commands[j].name) == 0){
            // Command Found!
            cmd.name = commands[j].name;
            cmd.commandNum = commands[j].commandNum;
            found = true;
            break;
        }
    }

    // Error Check
    if(!found){
        ESP_LOGE(TASK_NAME_COMMUNICATION, "Invalid User Command - Command not found");
        return {-1};
    }
    
    // Now we need to parse the parameters
    int paramNum;
    int paramFound = false;
    int paramBuffPointer = 0;
    char* paramEndPtr;
    for(paramNum = 0; paramNum < commands[j].numParams; paramNum++){
        paramFound = false;
        for(; i < 64; i++){
            switch (buffer[i]){
                // If we run into a space, skip it
                case ' ': 
                    break;

                // If we run into a comma, that marks the end of a parameter
                case ',':
                    paramBuff[paramBuffPointer] = '\0'; // Close up string
                    cmd.params[paramNum] = std::strtod(paramBuff, &paramEndPtr); // Converts paramBuff into a double and saves
                    if(*paramEndPtr != '\0'){
                        ESP_LOGE(TASK_NAME_COMMUNICATION, "Invalid User Command - Double Conversion Failed");
                        return {-1};
                    }
                    paramBuffPointer = 0;
                    i++; // to move onto the next character
                    paramFound = true;
                    break;

                // If we run into a ')', that marks the end of the function (last parameter)
                case ')':
                    // Checks to ensure we're on the the last parameter
                    if(paramNum != commands[j].numParams - 1){
                        ESP_LOGE(TASK_NAME_COMMUNICATION, "Invalid User Command - Invalid Parameter Count");
                        return {-1};
                    }
                    paramBuff[paramBuffPointer] = '\0'; // Close up string
                    cmd.params[paramNum] = std::strtod(paramBuff, &paramEndPtr); // Converts paramBuff into a double and saves
                    if(*paramEndPtr != '\0'){
                        ESP_LOGE(TASK_NAME_COMMUNICATION, "Invalid User Command - Double Conversion Failed");
                        return {-1};
                    }
                    paramFound = true;
                    break;

                // Otherwise, save the character
                default:
                    paramBuff[paramBuffPointer] = buffer[i];
                    paramBuffPointer++;
            } 
            // If we found the param, we move onto the next param loop
            if (paramFound){break;}
        }
    }

    return cmd;
}

// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// ~~ Task Definition ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

void CommunicationTask::taskEntry(void* pvParameters){
    CommunicationTask* self = static_cast<CommunicationTask*>(pvParameters);
    self->communicationTask();
}
/*
This task is responsible for managing and decoding all external communcations with the robotic arm. This includes
any bluetooth or UART control commands. These commands are parsed and sent to both the control task and the
kinematics task
*/
void CommunicationTask::communicationTask(){

}

// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// ~~ Other Utils ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

// Sends command over to control and kinematics task
void CommunicationTask::exportUserCommand(UserCommand* cmd){
    xQueueSend(controlCmd, cmd, portMAX_DELAY);
    xQueueSend(kinematicsCmd, cmd, portMAX_DELAY);
}


// *********************************** PUBLIC **************************************

// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// ~~ Task Controls ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

// Starts the task
void CommunicationTask::start(){
    if(taskHandle != nullptr){
        ESP_LOGE(TASK_NAME_COMMUNICATION, "Task is already running, returning...");
        return;
    }
    xTaskCreate(&CommunicationTask::taskEntry, TASK_NAME_COMMUNICATION, TASK_STACK_DEPTH_COMMUNICATION, NULL, TASK_PRIORITY_COMMUNICATION, &taskHandle);
    ESP_LOGI(TASK_NAME_COMMUNICATION, "Task started");
}

// Stops the task
void CommunicationTask::stop(){
    vTaskDelete(taskHandle);
    ESP_LOGI(TASK_NAME_COMMUNICATION, "Task Stopped");
}

// Restarts the task, if it's running
void CommunicationTask::restart(){
    if(taskHandle != nullptr){
        stop();
    }
    start();
}