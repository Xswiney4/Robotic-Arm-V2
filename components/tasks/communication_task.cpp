// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// ~~ Libraries ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
#include "robotic_definitions.h"
#include "communication_task.h"

// ESP/FreeRTOS
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

// Utils
#include <cstring>
#include <cstdlib>

// Bluetooth


// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// ~~ Definitions ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

static const char *communicationsTag = "Control Task";

// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// ~~ Initializations ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

void initBluetooth(){

}

void initUART(){

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

UserCommand userCmdParser(char *buffer){
    
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
        ESP_LOGE(communicationsTag, "Invalid User Command - Could not find '(' within string");
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
        ESP_LOGE(communicationsTag, "Invalid User Command - Command not found");
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
                        ESP_LOGE(communicationsTag, "Invalid User Command - Double Conversion Failed");
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
                        ESP_LOGE(communicationsTag, "Invalid User Command - Invalid Parameter Count");
                        return {-1};
                    }
                    paramBuff[paramBuffPointer] = '\0'; // Close up string
                    cmd.params[paramNum] = std::strtod(paramBuff, &paramEndPtr); // Converts paramBuff into a double and saves
                    if(*paramEndPtr != '\0'){
                        ESP_LOGE(communicationsTag, "Invalid User Command - Double Conversion Failed");
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

/*
This task is responsible for managing and decoding all external communcations with the robotic arm. This includes
any bluetooth or UART control commands. These commands are parsed and sent to both the control task and the
kinematics task
*/
void communicationsTask(void *pvParameter){

}

// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// ~~ Other Utils ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

// Sends command over to control and kinematics task
void exportUserCommand(UserCommand* cmd){
    xQueueSend(controlCmd, cmd, portMAX_DELAY);
    xQueueSend(kinematicsCmd, cmd, portMAX_DELAY);
}